import PyQt5
import sys
import cv2
import subprocess
import os
from pathlib import Path

# Hack to make PyQt and cv2 load simultaneously
os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = os.fspath(
    Path(PyQt5.__file__).resolve().parent / "Qt5" / "plugins"
)
import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets


class CameraWidget(QtWidgets.QWidget):
    new_frame = QtCore.pyqtSignal(np.ndarray)

    def __init__(self, parent=None):
        super().__init__(parent)

        self._cameras = self._get_connected_cameras()
        self._camera = None
        self._paused = False

        self._combo_box = QtWidgets.QComboBox(self)
        for device, name in self._cameras:
            self._combo_box.addItem(name, userData=device)
        self._combo_box.currentIndexChanged.connect(self._change_camera)

        self._start_button = QtWidgets.QPushButton("Start Video", self)
        self._start_button.clicked.connect(self._start_video)

        self._pause_button = QtWidgets.QPushButton("Pause", self)
        self._pause_button.setCheckable(True)
        self._pause_button.clicked.connect(self._pause_video)

        self._display_label = QtWidgets.QLabel(self)
        self._display_label.setMinimumSize(640, 480)
        self._display_label.setScaledContents(True)

        layout = QtWidgets.QVBoxLayout(self)
        top_layout = QtWidgets.QHBoxLayout()
        top_layout.addWidget(self._combo_box)
        top_layout.addWidget(self._start_button)
        top_layout.addWidget(self._pause_button)
        layout.addLayout(top_layout)
        layout.addWidget(self._display_label)

        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self._update_frame)

        self._capture_thread = QtCore.QThread(self)
        self._capture_thread.started.connect(self._capture_frames)
        self.new_frame.connect(self._update_display)
        self.new_frame.connect(self.np_frame)

    def _get_connected_cameras(self):
        cameras = []
        output = subprocess.check_output(["v4l2-ctl", "--list-devices"]).decode("utf-8")
        lines = output.strip().split("\n")
        i = 0
        while i < len(lines):
            line = lines[i].strip()
            if line.endswith(":"):
                camera_name = line[:-1]
                i += 1
                while i < len(lines) and not lines[i].startswith("\t"):
                    i += 1
                if i < len(lines):
                    device_path = lines[i].strip()
                    cameras.append((device_path, camera_name))
            else:
                i += 1
        return cameras

    def _change_camera(self, index):
        if self._camera is not None:
            self._camera.release()
        device_path = self._cameras[index][0]
        self._camera = cv2.VideoCapture(device_path)

    def _start_video(self):
        if self._camera is None:
            device_path = self._cameras[0][0]
            self._camera = cv2.VideoCapture(device_path)
        self._camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self._camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self._timer.start(33)  # 30 fps

    def _pause_video(self):
        self._paused = not self._paused

    def _update_frame(self):
        ret, frame = self._camera.read()
        if ret:
            self.new_frame.emit(frame)

    def _update_display(self, frame):
        if not self._paused:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            image = QtGui.QImage(frame.data, frame.shape[1], frame.shape[0],
                                 QtGui.QImage.Format_RGB888)
            pixmap = QtGui.QPixmap.fromImage(image)
            self._display_label.setPixmap(pixmap)

    def _capture_frames(self):
        if self._camera:
            while True:
                ret, frame = self._camera.read()
                if ret:
                    self.new_frame.emit(frame)

    def np_frame(self, frame):
        return np.asarray(frame)

    def start_capture_thread(self):
        self.moveToThread(self._capture_thread)
        self._capture_thread.start()


app = QtWidgets.QApplication(sys.argv)
widget = CameraWidget()
widget.start_capture_thread()
widget.show()
sys.exit(app.exec_())