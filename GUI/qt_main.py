from threading import Thread, Lock
from time import sleep

import serial
from PyQt5 import uic
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtWidgets import QApplication, QPushButton, QHBoxLayout, QWidget, QLabel, QSlider, QSpacerItem, QSizePolicy, \
    QLineEdit, QVBoxLayout
from pyrender import Renderer
from PyQt5.QtOpenGL import *
from urdfpy import URDF
import pyrender
import numpy as np
import trimesh
from IK import InverseIK, b2a, a2b

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
        sphere = trimesh.creation.icosphere(subdivisions=3, radius=0.01)
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


class App(QApplication):
    def __init__(self):
        super().__init__([])
        Form, Window = uic.loadUiType("ControlGUI.ui")
        self.window = Window()

        self.form = Form()
        self.form.setupUi(self.window)

        # self.form.openGLWidget.paintGL = types.MethodType(paintGL, self.form.openGLWidget)
        # self.form.openGLWidget.initializeGL = types.MethodType(initializeGL, self.form.openGLWidget)
        self.form.openGLWidget = WGL(self.form.splitter)
        self.form.openGLWidget.robot = URDF.load('/home/yves/tmp/urdfpy/tests/data/ur5/braccio.urdf')
        self.form.openGLWidget.updateScene({'elbow':0.5})
        self.form.openGLWidget.setSizePolicy(QSizePolicy.Expanding,QSizePolicy.Expanding)
        self.form.verticalLayout.insertWidget(0,self.form.openGLWidget)
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
            print(joint.name)

        for name, minv, maxv, default in [("Alpha", 0, 180, 90),
                                          ("Distance", 120, 250, 180),
                                          ("Hauteur", -80, 100, 50)]:
            slider = SliderWithLineEdit(name, minv, maxv, default)
            self.form.groupIK.layout().insertWidget(-1, slider)
            self.form.targetSlider[name] = slider
            slider.valueChanged.connect(self.updateTarget)
        self.form.groupIK.layout().addItem(QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Expanding))

        self.form.button_connect.clicked.connect(self.connect)
        self.ik = InverseIK()
        self.ik.init_braccio()

        self.serial = None
        self.serial_dev = None
        self.serial_lock = Lock()

        self.old_values = None
        self.updatePose()
        self.window.show()

    def updatePose(self, *args):
        cfg = {name: 3.15159*slider.value()/180.0 for name, slider in self.form.jointsSlider.items()}
        self.form.openGLWidget.updateScene(cfg)
        self.form.openGLWidget.update()

    def updateTarget(self):
        base = b2a(float(self.form.targetSlider["Alpha"].value()))
        dist = float(self.form.targetSlider["Distance"].value())
        z = float(self.form.targetSlider["Hauteur"].value())
        b,s,e,w = self.ik.solve_semipolar(base, dist, z)
        print(b, s, e, w)
        self.form.jointsSlider["base"].slider.setValue(int(a2b(b)))
        self.form.jointsSlider["shoulder"].slider.setValue(int(a2b(s)))
        self.form.jointsSlider["elbow"].slider.setValue(int(a2b(e)))
        self.form.jointsSlider["wrist_pitch"].slider.setValue(int(a2b(w)))

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

    def send(self):
        if self.serial is None:
            return
        values = [self.form.jointsSlider[i].value() for i in ['base',
                                                              'shoulder',
                                                              'elbow',
                                                              'wrist_pitch',
                                                              'wrist_roll',
                                                              'gripper_movable']] + [0,0]

        if self.old_values != values:
            self.send_values(values)

    def send_values(self, values):
        if self.serial is None:
            return
        if not self.serial_lock.acquire(True, 0.1):
            print("Timeout")
            return
        if self.serial.out_waiting == 0:
            print(" ".join([str(int(i)) for i in values]))
            b = bytes([255, ord('G')] + [int(x) for x in values])
            # b = bytes([255, ord('G')]) + bytes(" ".join([str(int(i)) for i in values])+" ", "utf-8")
            self.serial.write(b)
            self.serial.flush()
            self.old_values = values
        self.serial_lock.release()


class Ui_MainWindow(QWidget):
    def __init__(self, parent=None):
        super(Ui_MainWindow, self).__init__()
        self.GLview = WGL(None)
        self.GLview.robot = URDF.load('urdf/braccio.urdf')
        self.button = QPushButton('Test', self)
        mainLayout = QHBoxLayout()
        mainLayout.addWidget(self.GLview)
        mainLayout.addWidget(self.button)
        self.setLayout(mainLayout)


app = App()
app.exec_()