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

sg.theme('DarkAmber')   # Add a touch of color

class App:
    def __init__(self):
        self.labels = open("../datasets/TipCalib/labels/classes.txt").read().rstrip().split("\n")

        self.input_layout = [[sg.Button('Video')],
                            [sg.Button('YOLOv5'), sg.Button('Flow'), sg.Button('Auto-candidate'),
                                sg.Combo([f"{i} {self.labels[i]}" for i in range(len(self.labels))], key="-auto-candidate-class")],
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
        self.video_capture = None
        self.current_img = None
        self.current_np = None
        self.source = None
        self.model = torch.hub.load('../yolov5/', 'custom',
                               path='../runs/train/exp11/weights/best.pt',
                               source='local')

        self.figures = list()
        self.window = sg.Window('Vision UI (Ruby)', self.input_layout)
        event, values = self.window.read(16)
        self.graph = self.window['current_image']
        self.current_results = None

        self.running = True

    def save_candidate(self):
        l = list()
        for fn in os.listdir("candidates"):
            if fn.startswith("candidate_"):
                l.append(fn)
        last_ind = int(sorted(l)[-1].split(".")[-2].split("_")[-1])
        Image.fromarray(self.current_np).save(f"candidates/candidate_{last_ind+1}.png")

    def draw_results(self, g, res, selected_index=-1):
        for f in self.figures:
            self.graph.delete_figure(f)
        self.figures.clear()
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
            self.figures.append(g.draw_line((x, y1), (x, y2), color=color))
            self.figures.append(g.draw_line((x1, y), (x2, y), color=color))
            self.figures.append(g.draw_text(f"{int(r[5])} {r[4] * 100.0:.1f}", (x1, y1), color=color))

    def mainloop(self):
        processing = None
        refresh = True
        auto_candidates = False

        while self.running:
            event, values = self.window.read(16)
            if event == sg.WIN_CLOSED:
                break
            if event == 'Video':
                self.video_capture = cv2.VideoCapture(2)
                self.source = "video"
            if event == 'candidates_list':
                self.source = "file"
                self.current_np = np.array(Image.open("candidates/"+values["candidates_list"][0]))
                refresh = True
            if event == 'YOLOv5':
                processing = "YOLOv5"
                refresh = True
            if event == 'Flow':
                processing = "Flow"
                refresh = True
            if event == 'Auto-candidate':
                auto_candidates = not auto_candidates
            if self.source == "video":
                try:
                    _, cv2_im = self.video_capture.read()
                    self.current_np = cv2.cvtColor(cv2_im, cv2.COLOR_BGR2RGB)
                    refresh = True
                except Exception as e:
                    print(e)

            if event == "current_image":
                print(values["current_image"])

            if event == 'confidence_threshold':
                self.model.conf = values['confidence_threshold']
                refresh = True

            if event == '-roi-list':
                i = self.window.Element('-roi-list').Widget.curselection()
                if len(i)>0:
                    i=i[0]
                self.draw_results(self.graph, self.current_results, selected_index=i)

            if refresh:
                for f in self.figures:
                    self.graph.delete_figure(f)
                self.figures.clear()
                if self.current_np is not None:
                    self.graph.draw_img(self.current_np)

            if processing == "YOLOv5" and refresh:
                auto_candidate_class = -1
                if len(values["-auto-candidate-class"])>0:
                    auto_candidate_class = int(values["-auto-candidate-class"].split(" ")[0])
                self.model.conf = values['confidence_threshold']
                results = self.model(self.current_np)
                # draw = np.copy(current_np)

                roi_list = list()
                for r in results.xywh[0]:
                    s=f"{self.labels[int(r[5])]}: {int(r[0])} {int(r[1])} {int(r[2])} {int(r[3])}"
                    roi_list.append(s)
                self.window["-roi-list"].update(roi_list)
                self.current_results = list()
                for r in results.xywh[0]:
                    self.current_results.append(r.detach().tolist())

                self.draw_results(self.graph, self.current_results)
                for r in results.xywh[0]:
                    if auto_candidates \
                            and values["candidate_min"] < r[4] < values["candidate_max"] \
                            and int(r[5]) == auto_candidate_class:
                        print("Captured")
                        self.save_candidate(self.current_np)
                        self.window["candidates_list"].update(sorted(os.listdir("candidates/")))
                    # draw = cv2.rectangle(draw, (x1, y1), (x2,y2), (255,0,0))
                # current_img = ImageTk.PhotoImage(image=Image.fromarray(draw))

            refresh = False

a = App()
a.mainloop()