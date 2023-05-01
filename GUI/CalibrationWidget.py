import json
import sys
import unittest

from PyQt5 import QtCore
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QListWidget, QPushButton, QApplication
from PyQt5.QtCore import Qt
import bisect
from scipy.spatial import KDTree
import numpy as np

class CalibrationWidget(QWidget):
    add_button_clicked = QtCore.pyqtSignal()

    def __init__(self, settings_file='calibration.json'):
        super().__init__()

        self.matches = []
        self.settings_file = settings_file

        self.init_ui()
        self.load_matches()

    def init_ui(self):
        self.list_widget = QListWidget()
        self.add_btn = QPushButton("Add")
        self.add_btn.clicked.connect(self.on_add)
        self.delete_btn = QPushButton("Delete")
        self.delete_btn.clicked.connect(self.on_delete)

        hbox = QHBoxLayout()
        hbox.addWidget(self.add_btn)
        hbox.addWidget(self.delete_btn)

        vbox = QVBoxLayout()
        vbox.addWidget(self.list_widget)
        vbox.addLayout(hbox)

        self.setLayout(vbox)

    def save_matches(self):
        with open(self.settings_file, 'w') as f:
            json.dump(self.matches, f)

    def load_matches(self):
        try:
            with open(self.settings_file, 'r') as f:
                self.matches = json.load(f)
                for point2d, point3d in self.matches:
                    self.list_widget.addItem(f"2D: {point2d} -> 3D: {point3d}")
        except FileNotFoundError:
            pass  # File not found, start with an empty list of matches

    def add_match(self, point2d, point3d):
        index = bisect.bisect_left(self.points2d(), point2d)
        self.matches.insert(index, (point2d, point3d))
        self.list_widget.insertItem(index, f"2D: {point2d} -> 3D: {point3d}")
        self.save_matches()

    def to_3d(self, point2d):
        if not self.matches:
            raise ValueError("No matches available for interpolation")

        points2d = np.array(self.points2d())
        points3d = np.array(self.points3d())
        kdtree = KDTree(points2d)
        distances, indices = kdtree.query(point2d, k=2)

        if distances[0] == 0:
            return points3d[indices[0]]

        weights = 1 / distances
        weights /= np.sum(weights)

        interpolated_point3d = np.dot(weights, points3d[indices])

        return interpolated_point3d

    def points2d(self):
        return [match[0] for match in self.matches]

    def points3d(self):
        return [match[1] for match in self.matches]

    def on_add(self):
        self.add_button_clicked.emit()

    def on_delete(self):
        current_row = self.list_widget.currentRow()
        if current_row >= 0:
            self.list_widget.takeItem(current_row)
            del self.matches[current_row]


class TestCalibrationWidget(unittest.TestCase):
    def setUp(self):
        self.widget = CalibrationWidget()
        self.widget.add_match((0, 0), (0, 0, 0))
        self.widget.add_match((1, 1), (1, 1, 1))
        self.widget.add_match((2, 2), (2, 2, 2))

    def test_to_3d_exact_match(self):
        point2d = (1, 1)
        expected_point3d = np.array([1, 1, 1])
        result_point3d = self.widget.to_3d(point2d)
        np.testing.assert_array_almost_equal(result_point3d, expected_point3d)

    def test_to_3d_interpolation(self):
        point2d = (0.5, 0.5)
        expected_point3d = np.array([0.5, 0.5, 0.5])
        result_point3d = self.widget.to_3d(point2d)
        np.testing.assert_array_almost_equal(result_point3d, expected_point3d)

    def test_to_3d_no_matches(self):
        empty_widget = CalibrationWidget()
        with self.assertRaises(ValueError):
            empty_widget.to_3d((0, 0))

if __name__ == '__main__':
    app = QApplication(sys.argv)

    unittest.main()