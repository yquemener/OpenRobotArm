import PyQt5
import os
from pathlib import Path
import cv2

# Hack to make PyQt and cv2 load simultaneously
os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = os.fspath(
    Path(PyQt5.__file__).resolve().parent / "Qt5" / "plugins"
)

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication, QMainWindow, QSizePolicy, QVBoxLayout, QWidget, QPushButton
from DetectionResultsWidget import DetectionWidget


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        # Set up the main window
        self.setWindowTitle("Detection Widget Example")
        self.setCentralWidget(QWidget())

        # Set up the layout for the main window
        layout = QVBoxLayout(self.centralWidget())
        layout.setContentsMargins(0, 0, 0, 0)
        self.detection_widget = detection_widget = DetectionWidget()
        layout.addWidget(detection_widget)

        # Set up the video stream
        self.video_capture = cv2.VideoCapture(0)
        self.video_timer = QTimer(self)
        self.video_timer.timeout.connect(self.update_video)
        self.video_timer.start(50)

        # Set up the object list
        obj_list = [('SEGMENT', 100, 100, 200, 200),
                    ('POINT', 300, 300),
                    ('BOX', 400, 200, 100, 50)]
        self.detection_widget.set_detection(obj_list)

    def update_video(self):
        # Grab a frame from the video stream and set it as the background image
        ret, frame = self.video_capture.read()
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        if ret:
            self.detection_widget.set_frame(frame)


if __name__ == '__main__':
    app = QApplication([])
    window = MainWindow()
    window.show()
    app.exec_()
