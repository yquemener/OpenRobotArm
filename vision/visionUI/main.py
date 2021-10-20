import PySimpleGUI as sg
import torch
from PIL import Image, ImageTk
import io
import base64
import cv2
import numpy as np

"""
Function to add to PySimpleGUI's Graph class:


    def draw_img(self, img):
        # turn our ndarray into a bytesarray of PPM image by adding a simple header:
        # this header is good for RGB. for monochrome, use P5 (look for PPM docs)
        ppm = ('P6 %d %d 255 ' % (self.get_size()[0], self.get_size()[1])).encode('ascii') + img.tobytes()

        # turn that bytesarray into a PhotoImage object:
        image = tk.PhotoImage(width=self.get_size()[0], height=self.get_size()[1], data=ppm, format='PPM')

        # for first time, create and attach an image object into the canvas of our sg.Graph:
        if not hasattr(self, "img_id"):
            self.img_id = self.Widget.create_image((0, 0), image=image, anchor=tk.NW)
            # we must mimic the way sg.Graph keeps a track of its added objects:
            self.Images[self.img_id] = image
        else:
            # we reuse the image object, only changing its content
            self.Widget.itemconfig(self.img_id, image=image)
            # we must update this reference too: 
            self.Images[self.img_id] = image

"""



# def convert_to_bytes(file_or_bytes, resize=None):
#     '''
#     Will convert into bytes and optionally resize an image that is a file or a base64 bytes object.
#     Turns into  PNG format in the process so that can be displayed by tkinter
#     :param file_or_bytes: either a string filename or a bytes base64 image object
#     :type file_or_bytes:  (Union[str, bytes])
#     :param resize:  optional new size
#     :type resize: (Tuple[int, int] or None)
#     :return: (bytes) a byte-string object
#     :rtype: (bytes)
#     '''
#     if isinstance(file_or_bytes, str):
#         img = Image.open(file_or_bytes)
#     else:
#         try:
#             img = Image.open(io.BytesIO(base64.b64decode(file_or_bytes)))
#         except Exception as e:
#             dataBytesIO = io.BytesIO(file_or_bytes)
#             img = Image.open(dataBytesIO)
#
#     cur_width, cur_height = img.size
#     if resize:
#         new_width, new_height = resize
#         scale = min(new_height/cur_height, new_width/cur_width)
#         img = img.resize((int(cur_width*scale), int(cur_height*scale)), Image.ANTIALIAS)
#     bio = io.BytesIO()
#     img.save(bio, format="PNG")
#     del img
#     return bio.getvalue()


sg.theme('DarkAmber')   # Add a touch of color

input_layout = [[sg.Button('Video'), sg.Button('File')],
                [sg.Button('YOLOv5'), sg.Button('Flow')],
                [sg.Graph((640,480), (0,480), (640,0), key='current_image', enable_events=True, drag_submits=False),
                 sg.Slider(default_value=0.25, orientation="vertical", range=(0.01, 1.0),
                           resolution=0.01, key='confidence_threshold', enable_events=True)]]

filename = "/home/yves/Projects/active/HLA/OpenRobotArm/Ruby-Braccio/vision/datasets/TipCalib/images/calib-19.jpg"
video_capture = None
current_img = None
current_np = None
source = None
model = torch.hub.load('../yolov5/',
                       'custom',
                       path='../runs/train/exp11/weights/best.pt',
                       source='local')
figures = list()

if __name__ == '__main__':
    old_values = None
    window = sg.Window('Vision UI (Ruby)', input_layout)
    event, values = window.read(16)
    processing = None
    refresh = True

    while True:
        event, values = window.read(16)
        if event == sg.WIN_CLOSED:
            break
        if event == 'Video':
            video_capture = cv2.VideoCapture(0)
            source = "video"
        if event == 'File':
            source = "file"
            current_np = np.array(Image.open(filename))
            print(current_np.shape)
            current_img = ImageTk.PhotoImage(image=Image.fromarray(current_np))
            refresh = True
        if event == 'YOLOv5':
            processing = "YOLOv5"
            refresh = True
        if event == 'Flow':
            processing = "Flow"
            refresh = True
        if source == "video":
            try:
                _, cv2_im = video_capture.read()
                current_np = cv2.cvtColor(cv2_im, cv2.COLOR_BGR2RGB)
                # current_img = cv2.imencode('.png', cv2_im)[1].tobytes()
                current_img = ImageTk.PhotoImage(image=Image.fromarray(current_np))
                refresh = True
            except Exception as e:
                print(e)

        if event == "current_image":
            print(values["current_image"])

        if event == 'confidence_threshold':
            model.conf = values['confidence_threshold']
            refresh = True

        g = window['current_image']

        if refresh:
            for f in figures:
                g.delete_figure(f)
            if current_np is not None:
                g.draw_img(current_np)

        if processing == "YOLOv5" and refresh:
            model.conf = values['confidence_threshold']
            results = model(current_np)
            # draw = np.copy(current_np)

            for r in results.xywh[0]:
                x, y, w, h = [int(x) for x in r[:4].detach().tolist()]
                x1 = int(x - w / 2)
                y1 = int(y - h / 2)
                x2 = int(x + w / 2)
                y2 = int(y + h / 2)
                figures.append(g.draw_rectangle((x1,y1), (x2,y2), line_color="red"))
                # draw = cv2.rectangle(draw, (x1, y1), (x2,y2), (255,0,0))
            # current_img = ImageTk.PhotoImage(image=Image.fromarray(draw))

        refresh = False