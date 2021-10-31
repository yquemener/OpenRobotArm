import PySimpleGUI as sg
import torch
from PIL import Image, ImageTk
import io
import base64
import cv2
import numpy as np
import os

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

labels = open("../datasets/TipCalib/labels/classes.txt").read().rstrip().split("\n")

input_layout = [[sg.Button('Video')],
                [sg.Button('YOLOv5'), sg.Button('Flow'), sg.Button('Auto-candidate'),
                    sg.Combo([f"{i} {labels[i]}" for i in range(len(labels))], key="-auto-candidate-class")],
                [sg.Graph((640,480), (0,480), (640,0), key='current_image', enable_events=True, drag_submits=False),
                 sg.Slider(default_value=0.25, orientation="vertical", range=(0.01, 1.0),
                           resolution=0.01, key='confidence_threshold', enable_events=True),
                 sg.Column([[sg.Slider(default_value=0.75, orientation="vertical", range=(0.01, 1.0),
                                      resolution=0.01, key='candidate_max', enable_events=True)],
                            [sg.Slider(default_value=0.25, orientation="vertical", range=(0.01, 1.0),
                                      resolution=0.01, key='candidate_min', enable_events=True)]]),
                sg.Listbox(sorted(os.listdir("candidates/")), key="candidates_list", expand_y=True, enable_events=True),
                sg.Listbox([], key="-roi-list", expand_y=True, expand_x=True, enable_events=True),
                sg.Column([ [sg.Button("Remove", key="-remove-candidate")],
                            [sg.Button("Validate", key="-validate-candidate")],
                            [sg.Button("Up", key="-up")],
                            [sg.Button("Down", key="-down")],
                           ]),
                ]]

print(os.listdir("candidates/"))
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


def save_candidate(current_np):
    l=list()
    for fn in os.listdir("candidates"):
        if fn.startswith("candidate_"):
            l.append(fn)
    last_ind = int(sorted(l)[-1].split(".")[-2].split("_")[-1])
    print(last_ind)
    Image.fromarray(current_np).save(f"candidates/candidate_{last_ind+1}.png")


def draw_results(g, res, selected_index=-1):
    for f in figures:
        g.delete_figure(f)
    figures.clear()
    for i, r in enumerate(res):
        # x, y, w, h = [int(x) for x in r[:4].detach().tolist()]
        x, y, w, h, _, _ = r
        x1 = int(x - w / 2)
        y1 = int(y - h / 2)
        x2 = int(x + w / 2)
        y2 = int(y + h / 2)
        # figures.append(g.draw_rectangle((x1,y1), (x2,y2), line_color="red"))
        color = "black"
        if i == selected_index:
            color = "red"
        figures.append(g.draw_line((x, y1), (x, y2), color=color))
        figures.append(g.draw_line((x1, y), (x2, y), color=color))
        figures.append(g.draw_text(f"{int(r[5])} {r[4] * 100.0:.1f}", (x1, y1), color=color))


if __name__ == '__main__':
    old_values = None
    window = sg.Window('Vision UI (Ruby)', input_layout)
    event, values = window.read(16)
    processing = None
    refresh = True
    auto_candidates = False
    current_results = None

    while True:
        event, values = window.read(16)
        if event == sg.WIN_CLOSED:
            break
        if event == 'Video':
            video_capture = cv2.VideoCapture(2)
            source = "video"
        if event == 'candidates_list':
            source = "file"
            current_np = np.array(Image.open("candidates/"+values["candidates_list"][0]))
            print(current_np.shape)
            current_img = ImageTk.PhotoImage(image=Image.fromarray(current_np))
            refresh = True
        if event == 'YOLOv5':
            processing = "YOLOv5"
            refresh = True
        if event == 'Flow':
            processing = "Flow"
            refresh = True
        if event == 'Auto-candidate':
            auto_candidates = not auto_candidates
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

        if event == '-roi-list':
            i = window.Element('-roi-list').Widget.curselection()
            if len(i)>0:
                i=i[0]
            draw_results(g, current_results, selected_index=i)

        g = window['current_image']

        if refresh:
            for f in figures:
                g.delete_figure(f)
            figures.clear()
            if current_np is not None:
                g.draw_img(current_np)

        if processing == "YOLOv5" and refresh:
            auto_candidate_class = -1
            if len(values["-auto-candidate-class"])>0:
                auto_candidate_class = int(values["-auto-candidate-class"].split(" ")[0])
            model.conf = values['confidence_threshold']
            results = model(current_np)
            # draw = np.copy(current_np)

            roi_list = list()
            for r in results.xywh[0]:
                s=f"{labels[int(r[5])]}: {int(r[0])} {int(r[1])} {int(r[2])} {int(r[3])}"
                roi_list.append(s)
            window["-roi-list"].update(roi_list)
            current_results = list()
            for r in results.xywh[0]:
                current_results.append(r.detach().tolist())

            draw_results(g, current_results)
            for r in results.xywh[0]:
                if auto_candidates \
                        and values["candidate_min"] < r[4] < values["candidate_max"] \
                        and int(r[5]) == auto_candidate_class:
                    print("Captured")
                    save_candidate(current_np)
                    window["candidates_list"].update(sorted(os.listdir("candidates/")))
                # draw = cv2.rectangle(draw, (x1, y1), (x2,y2), (255,0,0))
            # current_img = ImageTk.PhotoImage(image=Image.fromarray(draw))

        refresh = False