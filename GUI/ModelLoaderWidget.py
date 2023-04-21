import os
import sys
import numpy as np
from pathlib import Path

import torch
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton,
    QVBoxLayout, QHBoxLayout, QListWidget, QListWidgetItem,
    QFileDialog, QMessageBox, QSizePolicy
)
from torchvision.transforms import transforms
from torchvision.transforms.functional import to_pil_image


class ModelLoaderWidget(QWidget):
    def __init__(self):
        super().__init__()

        # Default directory
        self.current_dir = "/home/yves/Projects/active/HLA/OpenArmVision/models"
        # self.current_dir = "/home/yves/Projects/active/HLA/OpenArmVision/yolov5/runs/train"

        # Widgets for selected dir and change folder button
        self.selected_dir_label = QLabel(self.current_dir)
        self.selected_dir_label.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
        self.change_folder_button = QPushButton("Change Folder")
        self.change_folder_button.clicked.connect(self.open_folder_dialog)
        self.model_state_label = QLabel("No model loaded")

        # Widget for list of .bin files
        self.file_list = QListWidget()
        self.file_list.itemSelectionChanged.connect(self.update_selected_file)

        # Widget for load model button
        self.load_model_button = QPushButton("Load Inference Model")
        self.load_model_button.clicked.connect(self.load_model)

        # Layout widgets vertically
        main_layout = QVBoxLayout()
        main_layout.addWidget(self.selected_dir_label)
        folder_layout = QHBoxLayout()
        folder_layout.addWidget(QLabel("Current Folder:"))
        folder_layout.addWidget(self.selected_dir_label)
        folder_layout.addWidget(self.change_folder_button)
        main_layout.addLayout(folder_layout)
        main_layout.addWidget(QLabel("Select a model file to load:"))
        main_layout.addWidget(self.file_list)
        main_layout.addWidget(self.load_model_button)
        main_layout.addWidget(self.model_state_label)
        self.setLayout(main_layout)

        # Load list of .bin files in the default directory
        self.update_file_list()

        # Member variable for last loaded model
        self.loaded_model = None
        self.model_type = None

    def open_folder_dialog(self):
        """Open a folder dialog to select a new folder."""
        folder = QFileDialog.getExistingDirectory(
            self, "Select Folder", self.current_dir, QFileDialog.ShowDirsOnly
        )
        if folder:
            self.current_dir = folder
            self.selected_dir_label.setText(self.current_dir)
            self.update_file_list()

    def update_file_list(self):
        """Update the list of .bin files."""
        self.file_list.clear()
        for filename in sorted(os.listdir(self.current_dir)):
            if filename.endswith(".bin") or filename.endswith(".pt"):
                item = QListWidgetItem(filename)
                self.file_list.addItem(item)

    def update_selected_file(self):
        """Update the selected file when it changes."""
        selected_items = self.file_list.selectedItems()
        if selected_items:
            self.selected_file = selected_items[0].text()

    def load_model(self):
        """Attempt to load the selected model file."""
        if hasattr(self, "selected_file"):
            filepath = os.path.join(self.current_dir, self.selected_file)
            try:
                if self.selected_file.endswith(".bin"):
                    self.loaded_model = torch.load(filepath)
                    self.model_type = "multi_obj"
                elif filepath.endswith(".pt"):
                    self.loaded_model = torch.hub.load(
                        '/home/yves/Projects/active/HLA/OpenArmVision/yolov5/', 'custom',
                        path=filepath,
                        source='local')
                    self.model_type = "YOLO"
                self.model_state_label.setText(Path(filepath).name + " loaded")
            except:
                QMessageBox.warning(
                    self, "Error", "Unable to load model from selected file."
                )

    def model(self):
        """Return the last loaded model."""
        return self.loaded_model

    def process_image(self, image, threshold=0.1):
        results = list()
        if self.model_type == "YOLO":
            self.loaded_model.eval()
            with torch.no_grad():
                res = self.loaded_model(image).xyxy[0]
            for r in res:
                results.append(["SEGMENT",
                                r[0], r[1], r[0]+640*r[2], r[1]+480*r[3]])

        elif self.model_type == "multi_obj":
            imgt = transforms.Resize((512, 512))(to_pil_image(image))
            imgt = torch.Tensor(np.array(imgt)).unsqueeze(0).view(1, 3, 512, 512).cuda()
            self.loaded_model.eval()
            with torch.no_grad():
                res = self.loaded_model(imgt)
            for i in range(res.shape[1]):
                for j in range(res.shape[2]):
                    arr = res[0, i, j].detach().tolist()
                    if arr[0] < threshold:
                        continue
                    arr = arr[1:]
                    arr[0] = (arr[0] + j + 0.5) * 640 / res.shape[1]
                    arr[1] = (arr[1] + i + 0.5) * 480 / res.shape[2]
                    arr[2] = arr[0] + arr[2] * 640
                    arr[3] = arr[1] + arr[3] * 480
                    results.append(["SEGMENT"] + arr)
        return results


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = QWidget()
    model_loader = ModelLoaderWidget()
    main_layout = QVBoxLayout()
    main_layout.addWidget(model_loader)
    window.setLayout(main_layout)
    window.show()
    sys.exit(app.exec_())
