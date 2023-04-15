import copy
import cv2
import numpy as np
import torch
import torch.functional
import torch.nn as nn
import torchvision.models as models
from torchvision.models import mobilenet_v2
import os
import torch
from torch.utils.data import Dataset
from PIL import Image
import torchvision.transforms as transforms
from torch.nn.functional import leaky_relu


class CustomYOLO(nn.Module):
    def __init__(self, num_classes, grid_size=16):
        super().__init__()
        self.num_classes = num_classes
        self.grid_size = grid_size
        self.output_channels = (1 + num_classes + 4)  # Objectness + Classes + 2 points (x1, y1, x2, y2)

        self.feature_extractor = self._build_feature_extractor()

        self.grid_small = 512
        self.grid_mid = 256
        self.grid_big = 256
        self.fc = 128
        self.fc_count = 2
        self.conv = nn.Conv2d(1280, self.grid_small, 1)     # 16x16
        self.conv2 = nn.Conv2d(1280, self.grid_mid, 2, 2)    # 8x8
        self.conv3 = nn.Conv2d(self.grid_mid, self.grid_big, 2, 2)     # 4x4
        
        self.ref = torch.nn.ModuleList()
        previous = self.grid_small + self.grid_mid + self.grid_big
        for i in range(self.fc_count):
          self.ref.append(nn.Conv2d(previous, self.fc, 1))
          previous = self.fc
        self.heads = nn.Conv2d(self.fc, self.output_channels, 1)
        # self.heads = nn.Conv2d(1280, self.output_channels, 1)

    def _build_feature_extractor(self):
        mobilenet = mobilenet_v2(pretrained=True)
        features = nn.Sequential(*list(mobilenet.children())[:-1])
        return features

    def forward(self, x):
        features = self.feature_extractor(x)
        features = torch.nn.functional.dropout(input=features, p=0.2)
        output1 = self.conv(features)
        output1 = leaky_relu(output1)            # 16x16

        output2 = self.conv2(features)      # 8x8
        output2 = leaky_relu(output2)

        output3 = self.conv3(output2)
        output3 = leaky_relu(output3)            # 4x4

        output2 = output2.repeat_interleave(2, dim=2).repeat_interleave(2, dim=3)
        output3 = output3.repeat_interleave(4, dim=2).repeat_interleave(4, dim=3)

        output = torch.cat((output1, output2, output3), dim=1)
        output = torch.nn.functional.dropout(input=output, p=0.2)
        for i in range(self.fc_count):
          output = self.ref[i](output)
          output = leaky_relu(output)
          # output = torch.nn.functional.dropout(input=output, p=0.2)
        output = self.heads(output)
        # output = self.heads(features)
        output = output.view(-1, self.grid_size, self.grid_size, self.output_channels)
        return output


def custom_yolo_loss(preds, targets, num_classes, alpha=1, beta=1, gamma=1):
    # Perte de classification
    class_loss = 0#  torch.nn.CrossEntropyLoss()(preds[..., 0].unsqueeze(-1), targets[..., 0].long())

    objectness_loss = torch.nn.MSELoss()(preds[..., 0], targets[..., 0])

    # Pertes de régression pour les points
    point1_loss = torch.nn.MSELoss()(preds[..., 1:3], targets[..., 1:3])
    point2_loss = torch.nn.MSELoss()(preds[..., 3:5], targets[..., 3:5])
    # point2_loss=0

    total_loss = alpha * class_loss + beta * (point1_loss + point2_loss*0.2) + gamma * objectness_loss
    return total_loss


def draw_segments(image, detections, threshold=0.5, line_thickness=2):
    img = image.copy()
    height, width, _ = img.shape
    print(detections.shape)
    grid_size = detections.shape[-1]

    for i in range(grid_size):
        for j in range(grid_size):
            objectness = detections[0, 0, i, j].item()
            if objectness > threshold:
                point1 = (int(detections[0, 1, i, j].item() * width), int(detections[0, 2, i, j].item() * height))
                point2 = (int(detections[0, 3, i, j].item() * width), int(detections[0, 4, i, j].item() * height))
                class_prob, class_id = torch.max(detections[0, 5:, i, j], 0)
                if class_prob.item() > threshold:
                    color = tuple(np.random.randint(0, 256, 3).tolist())
                    img = cv2.line(img, point1, point2, color, line_thickness)

    return img


class YOLODataset(Dataset):
    def __init__(self, img_dir, label_dir, num_classes, grid_size=16, transform=None):
        self.img_dir = img_dir
        self.label_dir = label_dir
        self.num_classes = num_classes
        self.grid_size = grid_size
        self.transform = transform

        self.image_files = sorted(os.listdir(img_dir))

    def __len__(self):
        return len(self.image_files)

    def __getitem__(self, idx):
        img_path = os.path.join(self.img_dir, self.image_files[idx])
        image = Image.open(img_path).convert('RGB')

        if self.transform:
            image = self.transform(image)

        label_path = os.path.join(self.label_dir, self.image_files[idx].replace('.jpg', '.txt'))
        label = torch.zeros(self.grid_size, self.grid_size, 1 + self.num_classes + 4)
        with open(label_path, 'r') as f:
            for line in f.readlines():
                cls, x1, y1, x2, y2 = map(float, line.strip().split(' '))
                i, j = int(y1 * self.grid_size), int(x1 * self.grid_size)
                xx1 = (x1 * self.grid_size) % 1 - 0.5
                yy1 = (y1 * self.grid_size) % 1 - 0.5
                label[i, j, 0] = 1
                label[i, j, 1:5] = torch.tensor([xx1, yy1, x2, y2])
                label[i, j, int(cls) + 5] = 1

        return image, label


def train(model, dataloader, optimizer, criterion, device, num_classes, num_epochs=10, filename=None):
    model.train()
    model.to(device)

    best_loss = 1e9
    for epoch in range(num_epochs):
        total_loss = 0
        for images, labels in dataloader:
            images = images.to(device)
            labels = labels.to(device)

            optimizer.zero_grad()
            preds = model(images)
            loss = criterion(preds, labels, num_classes)
            loss.backward()
            optimizer.step()

            total_loss += loss.item()
        l=total_loss / len(dataloader)
        saved = ""
        if filename is not None and epoch>10 and l<best_loss:
          best_loss = l
          torch.save(model, filename)
          saved = "*"

        print(f"Epoch {epoch + 1}/{num_epochs}, Loss: {total_loss / len(dataloader)} {saved}")
