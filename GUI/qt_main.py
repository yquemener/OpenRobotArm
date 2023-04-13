import serial
from PyQt5 import uic
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QApplication, QPushButton, QHBoxLayout, QWidget, QLabel, QSlider, QSpacerItem, QSizePolicy, \
    QLineEdit
from pyrender import Renderer
from PyQt5.QtOpenGL import *
from urdfpy import URDF
import pyrender
import numpy as np


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

class WGL(QGLWidget):
    def __init__(self, parent):
        QGLWidget.__init__(self, parent)

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
        self.form.jointsLineEdit = dict()
        for irow, joint in enumerate(self.form.openGLWidget.robot.actuated_joints):
            self.form.tabJoints.layout().addWidget(QLabel(f"{joint.name}"), irow, 0)
            slider = QSlider(Qt.Orientation.Horizontal)
            slider.valueChanged.connect(self.updatePose)
            slider.setMinimum(int(180*joint.limit.lower/np.pi))
            slider.setMaximum(int(180*joint.limit.upper/np.pi))
            self.form.jointsSlider[joint.name] = slider
            self.form.tabJoints.layout().addWidget(slider, irow, 1)

            lEdit = QLineEdit()
            lEdit.setMaximumSize(40, 1000)
            lEdit.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
            self.form.jointsLineEdit[joint.name] = lEdit
            self.form.tabJoints.layout().addWidget(lEdit, irow, 2)
            print(joint.name)
        self.form.tabJoints.layout().addItem(QSpacerItem(1,1,QSizePolicy.Minimum, QSizePolicy.Expanding), irow+1, 1)
        self.form.button_connect.clicked.connect(self.connect)
        self.window.show()

    def updatePose(self, *args):
        cfg = {name: 3.15159*slider.value()/180.0 for name, slider in self.form.jointsSlider.items()}
        for name, slider in self.form.jointsSlider.items():
            self.form.jointsLineEdit[name].setText(f"{slider.value()}")
        self.form.openGLWidget.updateScene(cfg)
        self.form.openGLWidget.update()

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
            if not self.amperes_thread_running:
                self.amperes_thread_running = True
                self.amperes_thread.start()
            self.form.label_connectionStatus.setText(f"Connected on \n{self.serial_dev}")
            self.serial.timeout = 0.1

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







# r.show(cfg={'shoulder':1.5, 'elbow':0.5})
# r.animate(cfg_trajectory={'shoulder' : [0, 1.5],'elbow' : [0, 1.5],})
