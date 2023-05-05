import math
from threading import Thread, Lock
from time import sleep

import torch
from PIL import Image
from skimage.transform import resize

import serial
from PyQt5 import uic
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtWidgets import QApplication, QPushButton, QHBoxLayout, QWidget, QLabel, QSlider, QSpacerItem, QSizePolicy, \
    QLineEdit, QVBoxLayout, QMainWindow, QSplitter, QFrame, QGroupBox, QTabWidget, QRadioButton, QButtonGroup
from pyrender import Renderer
from PyQt5.QtOpenGL import *
from torchvision.transforms import transforms
from urdfpy import URDF
import pyrender
import numpy as np
import trimesh

from IK import InverseIK, d2r, r2d
from VideoWidget import CameraWidget
from ModelLoaderWidget import ModelLoaderWidget
from DetectionResultsWidget import DetectionWidget
from CalibrationWidget import CalibrationWidget
from ThreadedController import ThreadedController

soft_start_pose = {
    'base': 90,
    'shoulder': 45,
    'elbow': 180,
    'wrist_pitch': 0,
    'wrist_roll': 90,
    'gripper_movable': 73
}

def compute_initial_camera_pose(scene):
    # centroid = scene.centroid
    scale = 0.5

    s2 = 1.0 / np.sqrt(2.0)
    cp = np.eye(4)
    cp[:3, :3] = np.array([
        [0.0, -s2, s2],
        [1.0, 0.0, 0.0],
        [0.0, s2, s2]
    ])
    hfov = np.pi / 6.0
    dist = scale / (2.0 * np.tan(hfov))
    cp[:3, 3] = dist * np.array([1.0, 0.0, 1.0]) #+ centroid
    return cp


def create_direct_light():
    light = pyrender.DirectionalLight(color=np.ones(3), intensity=1.0)
    n = pyrender.Node(light=light, matrix=np.eye(4))
    return n


class SliderWithLineEdit(QWidget):
    valueChanged = pyqtSignal(float)

    def __init__(self, value_text, min_value, max_value, initial_value=None):
        super().__init__()

        self.value_text = value_text
        self.min_value = min_value
        self.max_value = max_value

        # Create the slider
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMinimum(self.min_value)
        self.slider.setMaximum(self.max_value)
        self.slider.setTickPosition(QSlider.TicksBelow)
        self.slider.valueChanged.connect(self.on_value_changed)

        # Create the label
        self.label = QLabel(value_text)

        # Create the line edit
        self.line_edit = QLineEdit()
        self.line_edit.setMaximumWidth(50)
        self.line_edit.editingFinished.connect(self.update_slider)

        if initial_value is not None:
            self.slider.setValue(initial_value)
            self.line_edit.setText(str(initial_value))

        # Create the layouts
        slider_layout = QHBoxLayout()
        slider_layout.addWidget(self.label)
        slider_layout.addWidget(self.slider)
        slider_layout.addWidget(self.line_edit)

        main_layout = QVBoxLayout()
        main_layout.addLayout(slider_layout)

        self.setLayout(main_layout)

    def on_value_changed(self, value):
        self.line_edit.setText(str(value))
        self.valueChanged.emit(float(value))

    def update_slider(self):
        try:
            new_value = float(self.line_edit.text())
            if self.min_value <= new_value <= self.max_value:
                self.slider.setValue(int(new_value))
        except ValueError:
            pass

    def value(self):
        return float(self.slider.value())

    def setValue(self, val):
        self.slider.setValue(val)


class WGL(QGLWidget):
    def __init__(self, parent, target_position=(0.2, 0, 0)):
        QGLWidget.__init__(self, parent)
        self.target_position = np.array(target_position)

    def paintGL(self):
        self.renderer.render(self.scene, 0)

    def initializeGL(self):
        self.renderer = Renderer(800, 600)

    def resizeGL(self, w, h):
        self.renderer = Renderer(w, h)
        self.renderer.viewport_width = w
        self.renderer.viewport_height = h

    def updateScene(self, cfg):
        fk = self.robot.visual_trimesh_fk(cfg=cfg)
        self.scene = pyrender.Scene()
        for tm in fk:
            pose = fk[tm]
            mesh = pyrender.Mesh.from_trimesh(tm, smooth=False)
            self.scene.add(mesh, pose=pose)

        # Add a red sphere at target_position
        sphere = trimesh.creation.icosphere(subdivisions=3, radius=0.02)
        sphere.visual.vertex_colors = [255, 0, 0, 255]
        sphere_mesh = pyrender.Mesh.from_trimesh(sphere)
        sphere_node = pyrender.Node(mesh=sphere_mesh, matrix=np.eye(4))
        self.scene.add_node(sphere_node)
        pose = np.eye(4)
        pose[:3, 3] = self.target_position
        self.scene.set_pose(sphere_node, pose)

        self.scene._default_persp_cam = pyrender.PerspectiveCamera(
            yfov=np.pi / 3.0, znear=0.05, zfar=100.
        )
        self.scene._default_camera_pose = compute_initial_camera_pose(self.scene)
        camera = self.scene._default_persp_cam
        _camera_node = pyrender.Node(
            matrix=self.scene._default_camera_pose, camera=camera
        )

        self.scene.add_node(_camera_node)
        self.scene.main_camera_node = _camera_node
        self.scene.add_node(create_direct_light())


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Arm Controller")
        self.setGeometry(0, 0, 812, 731)

        self.centralwidget = QWidget(self)
        self.setCentralWidget(self.centralwidget)

        self.verticalLayout_3 = QVBoxLayout(self.centralwidget)

        self.horizontalLayout = QHBoxLayout()
        self.verticalLayout_3.addLayout(self.horizontalLayout)

        self.button_connect = QPushButton("Connect", self.centralwidget)
        self.horizontalLayout.addWidget(self.button_connect)

        self.label_connectionStatus = QLabel("Disconnected", self.centralwidget)
        sizePolicy = QSizePolicy(QSizePolicy.Preferred, QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        self.label_connectionStatus.setSizePolicy(sizePolicy)
        self.horizontalLayout.addWidget(self.label_connectionStatus)

        self.horizontalSpacer = QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)
        self.horizontalLayout.addItem(self.horizontalSpacer)

        self.horizontalFrame = QFrame(self.centralwidget)
        self.verticalLayout_4 = QVBoxLayout(self.horizontalFrame)

        self.splitter = QSplitter(Qt.Horizontal, self.horizontalFrame)

        self.verticalLayoutWidget_2 = QWidget(self.splitter)
        self.verticalLayout_2 = QVBoxLayout(self.verticalLayoutWidget_2)

        self.groupJoints = QGroupBox("Joints", self.verticalLayoutWidget_2)
        sizePolicy = QSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        self.groupJoints.setSizePolicy(sizePolicy)
        self.verticalLayout_7 = QVBoxLayout(self.groupJoints)
        self.verticalLayout_2.addWidget(self.groupJoints)

        self.tabs_IK = QTabWidget(self.verticalLayoutWidget_2)
        self.tabs_IK.setCurrentIndex(0)
        self.tab_IK_semipolar = QWidget()
        self.tabs_IK.addTab(self.tab_IK_semipolar, "IK semipolar")
        self.verticalLayout_6 = QVBoxLayout(self.tab_IK_semipolar)

        self.tab_IK_XYZ = QWidget()
        self.tabs_IK.addTab(self.tab_IK_XYZ, "IK XYZ")
        self.verticalLayout_9 = QVBoxLayout(self.tab_IK_XYZ)

        self.verticalLayout_2.addWidget(self.tabs_IK)

        self.verticalLayoutWidget = QWidget(self.splitter)
        self.verticalLayout = QVBoxLayout(self.verticalLayoutWidget)

        self.GLparent = QFrame(self.verticalLayoutWidget)
        self.GLparent.setFrameShape(QFrame.StyledPanel)
        self.GLparent.setFrameShadow(QFrame.Raised)
        self.verticalLayout.addWidget(self.GLparent)

        self.verticalLayout_4.addWidget(self.splitter)
        self.verticalLayout_3.addWidget(self.horizontalFrame)


class App(QApplication):
    def __init__(self):
        super().__init__([])

        self.form = MainWindow()

        self.form.openGLWidget = WGL(self.form.GLparent)
        self.form.openGLWidget.robot = URDF.load('urdf/braccio.urdf')
        self.form.openGLWidget.updateScene({'elbow': 0.5})
        self.form.openGLWidget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        self.form.detectionWidget = DetectionWidget()
        self.form.detectionWidget.test_click.connect(self.on_test_click)

        self.form.cameraWidget = CameraWidget()
        self.form.cameraWidget.new_frame.connect(self.on_new_frame)

        # Add checkable buttons
        self.form.mode_buttons = QButtonGroup()
        self.form.mode_button_layout = QHBoxLayout()

        for mode in ["Detection", "Calibration", "Acquisition", "Training", "Test"]:
            button = QRadioButton(mode)
            if mode == "Detection":
                button.setChecked(True)  # This line selects the "Detection" button by default
            self.form.mode_buttons.addButton(button)
            self.form.mode_button_layout.addWidget(button)
            button.toggled.connect(self.on_mode_changed)

        self.form.verticalLayout.insertWidget(0, self.form.openGLWidget)
        self.form.verticalLayout.insertWidget(0, self.form.detectionWidget)
        self.form.verticalLayout.insertWidget(0, self.form.cameraWidget)
        self.form.verticalLayout.insertLayout(1, self.form.mode_button_layout)
        self.form.verticalLayout.setStretch(0, 0)
        self.form.verticalLayout.setStretch(1, 1)
        self.form.verticalLayout.setStretch(2, 1)
        self.form.verticalLayout.setStretch(3, 1)
        # self.form.openGLWidget.initializeGL()

        self.form.jointsSlider = dict()
        self.form.targetSlider = dict()

        for joint in self.form.openGLWidget.robot.actuated_joints:
            slider = SliderWithLineEdit(joint.name,
                                        int(180*joint.limit.lower/np.pi),
                                        int(180*joint.limit.upper/np.pi),
                                        soft_start_pose.get(joint.name, int(180*joint.limit.lower/np.pi)))
            slider.valueChanged.connect(self.updatePose)
            self.form.jointsSlider[joint.name] = slider
            self.form.groupJoints.layout().insertWidget(-1, slider)

        for name, minv, maxv, default in [("Alpha", 0, 180, 180),
                                          ("Distance", 120, 250, 180),
                                          ("Hauteur", -50, 100, 50),
                                          ("Phi", 0, 360, 180)]:
            slider = SliderWithLineEdit(name, minv, maxv, default)
            self.form.tab_IK_semipolar.layout().insertWidget(-1, slider)
            self.form.targetSlider[name] = slider
            slider.valueChanged.connect(self.updateTargetPolar)

        for name, minv, maxv, default in [("X", 0, 300, 0),
                                          ("Y", -200, 250, 150),
                                          ("Z", -80, 100, 50),
                                          ("Phi_xyz", 0, 360, 180)]:
            slider = SliderWithLineEdit(name, minv, maxv, default)
            self.form.tab_IK_XYZ.layout().insertWidget(-1, slider)
            self.form.targetSlider[name] = slider
            slider.valueChanged.connect(self.updateTargetXYZ)

        self.form.actions_button_layout = QHBoxLayout()

        self.form.pump_button = QPushButton("Magnet")
        self.form.pump_button.setCheckable(True)
        self.form.actions_button_layout.insertWidget(-1, self.form.pump_button)
        self.form.go_grab_button = QPushButton("Grab first")
        self.form.go_grab_button.clicked.connect(self.go_grab)
        self.form.actions_button_layout.insertWidget(-1, self.form.go_grab_button)

        self.form.debug_button = QPushButton("Debug")
        self.form.debug_button.clicked.connect(self.debug)
        self.form.actions_button_layout.insertWidget(-1, self.form.debug_button)

        self.form.verticalLayout_2.insertLayout(-1, self.form.actions_button_layout)

        # Create the QTabWidget
        self.form.tabWidget = QTabWidget()
        self.form.modelLoader = ModelLoaderWidget()
        self.form.calibrationWidget = CalibrationWidget()
        self.form.calibrationWidget.add_button_clicked.connect(self.on_calibration_match_added)
        self.form.calibrationWidget.list_widget.currentItemChanged.connect(self.on_calibration_select_point)

        # Add ModelLoaderWidget and CalibrationWidget as separate tabs
        self.form.tabWidget.addTab(self.form.modelLoader, "Model Loader")
        self.form.tabWidget.addTab(self.form.calibrationWidget, "Calibration")

        # Insert the tab widget into the layout
        self.form.verticalLayout_2.insertWidget(-1, self.form.tabWidget)
        self.form.verticalLayout_2.addItem(QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Expanding))

        self.form.button_connect.clicked.connect(self.connect)
        self.ik = InverseIK()
        self.ik.init_braccio()

        self.controller = ThreadedController(self.form)

        self.serial = None
        self.serial_dev = None
        self.serial_lock = Lock()

        self.old_values = None
        self.updatePose()
        self.mode = None
        self.on_mode_changed()
        self.form.show()

    def on_mode_changed(self):
        checked_button = self.form.mode_buttons.checkedButton()
        if checked_button:
            self.mode = checked_button.text()
            self.form.detectionWidget.mode = self.mode
            print(f"Mode changed to: {self.mode}")
            if self.mode == "Test":
                points = list()
                for p1 in [a[0] for a in self.form.calibrationWidget.matches]:
                    for p2 in [a[0] for a in self.form.calibrationWidget.matches]:
                        if p1 == p2:
                            continue
                        points.append([(p1[0]+p2[0])//2, (p1[1]+p2[1])//2])

                objects = list()
                for p1 in points:
                    for p2 in points:
                        objects.append(["SEGMENT", p1[0], p1[1], p2[0], p2[1]])

                self.form.detectionWidget.set_detection(objects)
        else:
            self.mode = None
            print("No mode selected")

    def on_new_frame(self, frame):
        np_frame = self.form.cameraWidget.np_frame()
        self.form.detectionWidget.set_frame(np_frame)
        if self.mode == "Detection":
            segments = self.form.modelLoader.process_image(np_frame)
            self.form.detectionWidget.set_detection(segments)

    def updatePose(self, *args):
        cfg = {name: 3.15159*slider.value()/180.0 for name, slider in self.form.jointsSlider.items()}
        self.form.openGLWidget.updateScene(cfg)
        self.form.openGLWidget.update()

    def updateTargetPolar(self):
        base = d2r(float(self.form.targetSlider["Alpha"].value()))
        dist = float(self.form.targetSlider["Distance"].value())
        z = float(self.form.targetSlider["Hauteur"].value())
        phi = float(self.form.targetSlider["Phi"].value())*math.pi/180.
        b, s, e, w = self.ik.solve_semipolar_iv(base, dist, z, phi=phi)

        self.form.jointsSlider["base"].slider.setValue(int(r2d(b)))
        self.form.jointsSlider["shoulder"].slider.setValue(int(r2d(s)))
        self.form.jointsSlider["elbow"].slider.setValue(int(r2d(e)))
        self.form.jointsSlider["wrist_pitch"].slider.setValue(int(r2d(w)))

    def updateTargetXYZ(self):
        x = float(self.form.targetSlider["X"].value())
        y = float(self.form.targetSlider["Y"].value())
        z = float(self.form.targetSlider["Z"].value())
        phi = float(self.form.targetSlider["Phi_xyz"].value()) * math.pi / 180.
        self.form.openGLWidget.target_position = np.array((x,-y,z))/1000.
        self.updatePose()
        b, s, e, w = self.ik.solve(x, y, z, phi=phi)

        self.form.jointsSlider["base"].slider.setValue(int(r2d(b)))
        self.form.jointsSlider["shoulder"].slider.setValue(int(r2d(s)))
        self.form.jointsSlider["elbow"].slider.setValue(int(r2d(e)))
        self.form.jointsSlider["wrist_pitch"].slider.setValue(int(r2d(w)))

    def connect(self):
        self.form.label_connectionStatus.setText("Connecting")
        success = False
        num = 0
        self.serial = serial.Serial(port=None, baudrate=57600)
        self.serial.dts = True
        while not success:
            try:
                self.serial.port = f'/dev/ttyACM{num}'
                self.serial.open()
                self.serial_dev = f'/dev/ttyACM{num}'
                success = True
                break
            except:
                print(f"Failed to connect to /dev/ttyACM{num}")
                if num > 100:
                    self.form.label_connectionStatus.setText("Disconnected")
                    break
            try:
                self.serial.port = f'/dev/ttyUSB{num}'
                self.serial.open()
                self.serial_dev = f'/dev/ttyUSB{num}'
                success = True
                break
            except:
                print(f"Failed to connect to /dev/ttyUSB{num}")
                num += 1
                if num > 100:
                    self.form.label_connectionStatus.setText("Disconnected")
                    break
        if not success:
            self.serial = None
        else:
            self.form.label_connectionStatus.setText(f"Connected on \n{self.serial_dev}")
            self.serial.timeout = 0.1
            Thread(target=self.send_thread_func).start()

    def send_thread_func(self):
        while self.form.label_connectionStatus.text().startswith("Connected"):
            sleep(0.1)
            self.send()


    def get_pose(self):
        pose = [self.form.jointsSlider[i].value() for i in ['base',
                                                            'shoulder',
                                                            'elbow',
                                                            'wrist_pitch',
                                                            'wrist_roll',
                                                            'gripper_movable']] + [0, 0]
        if self.form.pump_button.isChecked():
            pose[6] = 1
            pose[7] = 1
        return pose

    def debug(self):
        pose = self.get_pose()

        pose1 = pose[:]
        pose1[0] += 45
        pose1[6] = 1
        pose1[7] = 1
        pose2 = pose[:]
        pose2[6] = 0
        pose2[7] = 0


        instructions = \
            [["pose"]+pose1,
             ["sleep", 3],
             ["pose"]+pose2]
        self.controller.add_instructions(instructions)

    def send(self):
        if self.serial is None:
            return
        values = self.get_pose()

        if self.old_values != values:
            self.send_values(values)

    def send_values(self, values):
        if self.serial is None:
            return
        if not self.serial_lock.acquire(True, 0.1):
            print("Timeout")
            return
        if self.serial.out_waiting == 0:
            # print(" ".join([str(int(i)) for i in values]))
            b = bytes([255, ord('G')] + [int(x) for x in values])
            # b = bytes([255, ord('G')]) + bytes(" ".join([str(int(i)) for i in values])+" ", "utf-8")
            self.serial.write(b)
            self.serial.flush()
            self.old_values = values
        self.serial_lock.release()

    def on_calibration_match_added(self):
        ol = self.form.detectionWidget.obj_list
        if len(ol) > 0:
            p2d = (ol[0][1], ol[0][2])
            del ol[0]
            self.form.detectionWidget.set_detection()
        else:
            return
        p3d = (
            self.form.targetSlider["X"].value(),
            self.form.targetSlider["Y"].value(),
            self.form.targetSlider["Z"].value()
        )
        self.form.calibrationWidget.add_match(p2d, p3d)

    def on_calibration_select_point(self):
        print(self.form.calibrationWidget.matches)
        print(self.form.calibrationWidget.list_widget.currentRow())
        a = self.form.calibrationWidget.matches[self.form.calibrationWidget.list_widget.currentRow()]
        print(a)
        self.form.detectionWidget.set_detection([["POINT", a[0][0], a[0][1]]])

    def on_test_click(self, x, y):
        p3d = self.form.calibrationWidget.to_3d([x, y])
        print(x,y, p3d)
        self.form.targetSlider["X"].setValue(int(p3d[0]))
        self.form.targetSlider["Y"].setValue(int(p3d[1]))
        self.form.targetSlider["Z"].setValue(int(p3d[2]))



    def go_grab(self):
        ol = self.form.detectionWidget.obj_list
        if len(ol) < 1:
            return
        target2d = (ol[0][1], ol[0][2])
        t3d = self.form.calibrationWidget.to_3d(target2d)
        pose1 = self.ik.solve(t3d[0], t3d[1], 80, phi=math.pi)
        pose2 = self.ik.solve(t3d[0], t3d[1], t3d[2], phi=math.pi)
        pose1 = [int(r2d(a)) for a in pose1]
        pose2 = [int(r2d(a)) for a in pose2]

        instructions = []
        instructions.append(["pose"] + pose1 + [0,0,0,0])
        instructions.append(["sleep", 2])

        instructions.append(["pose"] + pose2 + [0,0,0,0])
        instructions.append(["sleep", 1])
        instructions.append(["pose"] + pose2 + [0,0,1,1])

        b, s, e, w = self.ik.solve(t3d[0], t3d[1], 80, phi=math.pi)
        instructions.append(["pose"] + pose1 + [0, 0, 1, 1])
        instructions.append(["sleep", 1])

        instructions.append(["pose"] + [pose1[0]+45]+pose1[1:] + [0,0,0,0])
        instructions.append(["sleep", 2])

        self.controller.add_instructions(instructions)


app = App()
app.exec_()