import json
import sys
import unittest
from math import sqrt

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

    # def to_3d(self, point2d):
    #     if not self.matches:
    #         raise ValueError("No matches available for interpolation")
    #
    #     points2d = np.array(self.points2d())
    #     points3d = np.array(self.points3d())
    #     kdtree = KDTree(points2d)
    #     distances, indices = kdtree.query(point2d, k=4)
    #
    #     if distances[0] == 0:
    #         return points3d[indices[0]]
    #
    #     weights = 1 / distances
    #     weights /= np.sum(weights)
    #
    #     interpolated_point3d = np.dot(weights, points3d[indices])
    #
    #     return interpolated_point3d

    def to_3d(self, point2d):
        if not self.matches:
            raise ValueError("No matches available for interpolation")

        points2d = np.array(self.points2d())
        points3d = np.array(self.points3d())
        kdtree = KDTree(points2d)
        distances, indices = kdtree.query(point2d, k=4)

        # Get the four closest points and their corresponding 3D points
        P1, P2, P3, P4 = points2d[indices]
        Q1, Q2, Q3, Q4 = points3d[indices]

        p2d = points2d[indices]

        tl = tr = bl = br = None
        for p, pp in zip(p2d, points3d[indices]):
            lp = np.sum(p2d[:, 0] > p[0])
            tp = np.sum(p2d[:, 1] > p[1])
            if lp > 1 and tp > 1:
                tl = p
                qtl=pp
            elif lp < 2 and tp > 1:
                tr = p
                qtr=pp
            elif lp > 1 and tp < 2:
                bl = p
                qbl = pp
            elif lp < 2 and tp < 2:
                br = p
                qbr = pp

        ca = tl-bl
        ab = tr-tl
        cd = br-bl
        cp = bl-point2d

        # polynome's factors:
        a = cd[1]*(ab[0]-cd[0]) + cd[0]*(ab[1]-cd[1])
        b = -cp[1]*(ab[0]-cd[0]) - cd[1]*ca[0]+cd[0]*ca[1] - cp[0]*(ab[1]-cd[1])
        c = cp[1]*ca[0] - cp[0]*ca[1]

        delta = b**2 - 4*a*c
        #delta = ca[0] ** 2 - 4 * (ab[0] - cd[0]) * (bl[0] - point2d[0])
        sol1x = (-b + sqrt(delta)) / (2 * a)
        sol2x = (-b - sqrt(delta)) / (2 * a)

        sol1y = (cp[0] - sol1x * cd[0]) / (ca[0] + sol1x * (ab[0] - cd[0]))
        sol2y = (cp[0] - sol2x * cd[0]) / (ca[0] + sol2x * (ab[0] - cd[0]))
        print(sol1x, sol1y)
        print(sol2x, sol2y)
        print()

        if(abs(sol1x)<=1.):
            solx = sol1x
            soly = sol1y
        elif(abs(sol2x)<=1.):
            solx = sol2x
            soly = sol2y
        else:
            print("No intrpolated solution")
            return

        print(tr,tl,br,bl)

        weight1 = (1-solx)*(1-soly)
        weight2 = solx*(1-soly)
        weight3 = (1-solx) * soly
        weight4 = solx * (1-soly)

        interpolated_point3d = weight1 * qtl + weight2 * qtr + weight3 * qbl + weight4 * qbr

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