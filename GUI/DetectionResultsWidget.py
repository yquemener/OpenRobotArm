import numpy as np
from PyQt5.QtCore import Qt, QRectF, QSize, QPoint
from PyQt5.QtGui import QBrush, QColor, QPen, QPixmap, QImage
from PyQt5.QtWidgets import QGraphicsScene, QGraphicsView, QListWidget, QListWidgetItem, QWidget, QGraphicsPixmapItem, \
    QGraphicsLineItem, QGraphicsRectItem, QHBoxLayout, QSplitter, QVBoxLayout
from PyQt5 import QtWidgets
from collections import defaultdict


class GraphicsScene(QGraphicsScene):
    def __init__(self, parent=None):
        QGraphicsScene.__init__(self, parent)

    def mousePressEvent(self, event):
        self.parent().on_click(event)


class DetectionWidget(QWidget):
    def __init__(self):
        super().__init__()

        # Set up the QGraphicsScene and QGraphicsView
        self.scene = GraphicsScene(self)
        self.view = QGraphicsView(self.scene, self)
        # self.view.setFixedSize(640, 480)

        # Set up the list widget
        self.list_widget = QListWidget(self)
        # self.list_widget.setFixedSize(150, 480)
        self.list_widget.itemSelectionChanged.connect(self.on_item_selection_changed)
        self.selected_obj_index = None

        # Set up the splitter to contain the list widget and graphics view
        self.splitter = QSplitter(Qt.Horizontal)
        self.splitter.addWidget(self.view)
        self.splitter.addWidget(self.list_widget)

        # Set the layout of the widget to the splitter
        layout = QVBoxLayout()
        layout.addWidget(self.splitter)
        self.setLayout(layout)

        # Initialize the background image and add it to the scene
        self.background_image = None
        self.background_pixmap = QPixmap(640, 480)
        self.background_pixmap.fill(Qt.black)
        self.background_item = self.scene.addPixmap(self.background_pixmap)

        self.obj_list = list()
        self.mode = "Detection"

    def set_frame(self, img):
        # Validate the input
        if not isinstance(img, np.ndarray):
            return
        if img.shape != (480, 640, 3):
            return
        # Convert the numpy array to a QImage and set it as the background image
        self.background_image = QImage(img.data, img.shape[1], img.shape[0], QImage.Format_RGB888)
        self.background_pixmap = QPixmap.fromImage(self.background_image)
        # Set the pixmap on the background item and fit it to the view
        self.background_item.setPixmap(self.background_pixmap)
        # self.view.fitInView(self.background_item, Qt.KeepAspectRatio)

    def set_detection(self, obj_list=None):
        if obj_list is not None:
            self.obj_list = obj_list
        self.list_widget.clear()
        for i, obj in enumerate(self.obj_list):
            self.list_widget.addItem(f"{obj[0]} {[int(a) for a in obj[1:]]}")
        self.update_detection_objects()

    def update_detection_objects(self):
        # Clear the detection objects from the scene and list widget
        for item in self.scene.items():
            if not isinstance(item, QGraphicsPixmapItem):
                self.scene.removeItem(item)

        # Set up the brushes and pens for drawing the objects
        default_pen = QPen(Qt.red)
        default_pen.setWidth(2)
        selected_pen = QPen(Qt.blue)
        selected_pen.setWidth(2)

        # Draw each object on the scene and add it to the list widget
        for i, obj in enumerate(self.obj_list):
            if i == self.selected_obj_index:
                pen = selected_pen
            else:
                pen = default_pen
            if obj[0] == 'SEGMENT':
                item = QGraphicsLineItem(obj[1], obj[2], obj[3], obj[4])
                item.setPen(pen)
                self.scene.addEllipse(obj[1] - 5, obj[2] - 5, 10, 10, pen.color())
                item.setPen(pen)
                self.scene.addItem(item)
            elif obj[0] == 'POINT':
                item = QGraphicsLineItem(obj[1] - 10, obj[2], obj[1] + 10, obj[2])
                item.setPen(pen)
                self.scene.addItem(item)
                item = QGraphicsLineItem(obj[1], obj[2] - 10, obj[1], obj[2] + 10)
                item.setPen(pen)
                self.scene.addItem(item)
            elif obj[0] == 'BOX':
                item = QGraphicsRectItem(obj[1] - obj[3] / 2, obj[2] - obj[4] / 2, obj[3], obj[4])
                item.setPen(pen)
                self.scene.addItem(item)

    def on_item_selection_changed(self):
        # Set the pen color of the selected item to blue
        selected_indices = self.list_widget.selectedIndexes()
        if len(selected_indices)>0:
            self.selected_obj_index = selected_indices[0].row()
        else:
            self.selected_obj_index = None
        self.update_detection_objects()

    def on_click(self, event):
        x = event.scenePos().x()
        y = event.scenePos().y()
        if self.mode == "Calibration":
            self.obj_list.append(["POINT", x, y])
            self.set_detection(self.obj_list)
