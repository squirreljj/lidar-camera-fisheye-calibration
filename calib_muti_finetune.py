# coding:utf-8
import math
import cv2
import numpy as np
# import glob
import sys
import open3d as o3d
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import (QApplication, QMainWindow, QLabel, QDoubleSpinBox)
np.set_printoptions(suppress=True)
import os

import matplotlib.pyplot as plt
np.set_printoptions(suppress=True)

img_points = []

lidar_points = []
class DemoDoubleSpinBox(QMainWindow):
    def __init__(self, parent=None):
        super(DemoDoubleSpinBox, self).__init__(parent)

        # 设置窗口标题
        self.setWindowTitle('fune tuning')
        # 设置窗口大小
        self.resize(400, 300)
        print('init')
        print(str(rx) + " " + str(ry) + " " + str(rz) + " " + str(x) + " " + str(y) + " " + str(z))
        # rx
        label_default = QLabel('rx', self)
        label_default.setGeometry(10, 10, 80, 24)
        label_default.setAlignment(QtCore.Qt.AlignRight)
        self.sb_default = QDoubleSpinBox(self)
        #global rx
        self.sb_default.setValue(float(rx))
        self.sb_default.setRange(-4, 4)
        self.sb_default.setGeometry(100, 10, 160, 26)
        self.sb_default.setDecimals(3)
        self.sb_default.setSingleStep(0.002)
        self.sb_default.valueChanged.connect(self.defaultSpinBoxValueChanged)

        # ry
        label_prefix = QLabel('ry', self)
        label_prefix.setGeometry(10, 50, 80, 26)
        label_prefix.setAlignment(QtCore.Qt.AlignRight)
        self.sb_prefix = QDoubleSpinBox(self)
        #global ry
        self.sb_prefix.setValue(float(ry))
        self.sb_prefix.setRange(-4, 4)
        self.sb_prefix.setGeometry(100, 50, 160, 26)
        self.sb_prefix.setDecimals(3)
        self.sb_prefix.setSingleStep(0.002)
        self.sb_prefix.valueChanged.connect(self.prefixSpinBoxTextChanged)

        # rz
        label_suffix = QLabel('rz', self)
        label_suffix.setGeometry(10, 90, 80, 26)
        label_suffix.setAlignment(QtCore.Qt.AlignRight)
        self.sb_suffix = QDoubleSpinBox(self)
        #global rz
        self.sb_suffix.setValue(float(rz))
        self.sb_suffix.setRange(-4, 4)
        self.sb_suffix.setGeometry(100, 90, 160, 26)
        self.sb_suffix.setDecimals(3)
        self.sb_suffix.setSingleStep(0.002)
        self.sb_suffix.valueChanged.connect(self.suffixSpinBoxTextChanged)

        # x
        label_x = QLabel('x', self)
        label_x.setGeometry(10, 130, 80, 26)
        label_x.setAlignment(QtCore.Qt.AlignRight)
        self.sb_x = QDoubleSpinBox(self)
        #global x
        self.sb_x.setValue(float(x))
        self.sb_x.setRange(-4, 4)
        self.sb_x.setGeometry(100, 130, 160, 26)
        self.sb_x.setDecimals(3)
        self.sb_x.setSingleStep(0.002)
        self.sb_x.valueChanged.connect(self.xSpinBoxValueChanged)

        # y
        label_y = QLabel('y', self)
        label_y.setGeometry(10, 170, 80, 26)
        label_y.setAlignment(QtCore.Qt.AlignRight)
        self.sb_y = QDoubleSpinBox(self)
        #global y
        self.sb_y.setValue(float(y))
        self.sb_y.setRange(-4, 4)
        self.sb_y.setGeometry(100, 170, 160, 26)
        self.sb_y.setDecimals(3)
        self.sb_y.setSingleStep(0.002)
        self.sb_y.valueChanged.connect(self.ySpinBoxTextChanged)

        # z
        label_z = QLabel('z', self)
        label_z.setGeometry(10, 210, 80, 26)
        label_z.setAlignment(QtCore.Qt.AlignRight)
        self.sb_z = QDoubleSpinBox(self)
        #global z
        self.sb_z.setValue(float(z))
        self.sb_z.setRange(-4,4)
        self.sb_z.setGeometry(100, 210, 160, 26)
        self.sb_z.setDecimals(3)
        self.sb_z.setSingleStep(0.002)
        self.sb_z.valueChanged.connect(self.zSpinBoxTextChanged)

        # 信息显示
        self.label_value = QLabel(self)
        self.label_value.setGeometry(10, 240, 280, 30)

    def defaultSpinBoxValueChanged(self, a0):
        self.label_value.setText('current value is:' + str(a0))
        global rx
        rx=a0
        print(str(rx) + " " + str(ry) + " " + str(rz) + " " + str(x) + " " + str(y) + " " + str(z))
        rr = np.array([[rx], [ry], [rz]])
        tt = np.array([[x], [y], [z]])
        img1 = cv2.imread("D:/fisheye+disparity2pointcloud/lidar_camera_calib/2_7/img/4.png")
        img2 = cv2.imread("D:/fisheye+disparity2pointcloud/lidar_camera_calib/2_7/img/5.png")
        img3 = cv2.imread("D:/fisheye+disparity2pointcloud/lidar_camera_calib/2_7/img/6.png")

        project_p(pcd1_path,pcd2_path,pcd3_path,img1,img2,img3, rr, tt, "1","2","3")

    def prefixSpinBoxTextChanged(self, a0):
        self.label_value.setText('current text is:' + str(a0))
        global ry
        ry = a0
        print(str(rx) + " " + str(ry) + " " + str(rz) + " " + str(x) + " " + str(y) + " " + str(z))
        rr = np.array([[rx], [ry], [rz]])
        tt = np.array([[x], [y], [z]])
        img1 = cv2.imread("D:/fisheye+disparity2pointcloud/lidar_camera_calib/2_7/img/4.png")
        img2 = cv2.imread("D:/fisheye+disparity2pointcloud/lidar_camera_calib/2_7/img/5.png")
        img3 = cv2.imread("D:/fisheye+disparity2pointcloud/lidar_camera_calib/2_7/img/6.png")
        # 输入三组点云的路径

        project_p(pcd1_path,pcd2_path,pcd3_path,img1,img2,img3, rr, tt, "1","2","3")
    def suffixSpinBoxTextChanged(self, a0):
        self.label_value.setText('current text is:' + str(a0))
        global rz
        rz = a0
        print(str(rx) + " " + str(ry) + " " + str(rz) + " " + str(x) + " " + str(y) + " " + str(z))
        rr = np.array([[rx], [ry], [rz]])
        tt = np.array([[x], [y], [z]])
        img1 = cv2.imread("D:/fisheye+disparity2pointcloud/lidar_camera_calib/2_7/img/4.png")
        img2 = cv2.imread("D:/fisheye+disparity2pointcloud/lidar_camera_calib/2_7/img/5.png")
        img3 = cv2.imread("D:/fisheye+disparity2pointcloud/lidar_camera_calib/2_7/img/6.png")
        # 输入三组点云的路径

        project_p(pcd1_path,pcd2_path,pcd3_path,img1,img2,img3, rr, tt, "1","2","3")
    def xSpinBoxValueChanged(self, a0):
        self.label_value.setText('current value is:' + str(a0))
        global x
        x = a0
        print(str(rx) + " " + str(ry) + " " + str(rz) + " " + str(x) + " " + str(y) + " " + str(z))
        rr = np.array([[rx], [ry], [rz]])
        tt = np.array([[x], [y], [z]])
        img1 = cv2.imread("D:/fisheye+disparity2pointcloud/lidar_camera_calib/2_7/img/4.png")
        img2 = cv2.imread("D:/fisheye+disparity2pointcloud/lidar_camera_calib/2_7/img/5.png")
        img3 = cv2.imread("D:/fisheye+disparity2pointcloud/lidar_camera_calib/2_7/img/6.png")
        # 输入三组点云的路径

        project_p(pcd1_path,pcd2_path,pcd3_path,img1,img2,img3, rr, tt, "1","2","3")
    def ySpinBoxTextChanged(self, a0):
        self.label_value.setText('current text is:' + str(a0))
        global y
        y = a0
        print(str(rx) + " " + str(ry) + " " + str(rz) + " " + str(x) + " " + str(y) + " " + str(z))
        rr = np.array([[rx], [ry], [rz]])
        tt = np.array([[x], [y], [z]])
        img1 = cv2.imread("D:/fisheye+disparity2pointcloud/lidar_camera_calib/2_7/img/4.png")
        img2 = cv2.imread("D:/fisheye+disparity2pointcloud/lidar_camera_calib/2_7/img/5.png")
        img3 = cv2.imread("D:/fisheye+disparity2pointcloud/lidar_camera_calib/2_7/img/6.png")
        # 输入三组点云的路径

        project_p(pcd1_path,pcd2_path,pcd3_path,img1,img2,img3, rr, tt, "1","2","3")
    def zSpinBoxTextChanged(self, a0):
        self.label_value.setText('current text is:' + str(a0))
        global z
        z = a0
        print(str(rx) + " " + str(ry) + " " + str(rz) + " " + str(x) + " " + str(y) + " " + str(z))
        rr = np.array([[rx], [ry], [rz]])
        tt = np.array([[x], [y], [z]])
        img1 = cv2.imread("D:/fisheye+disparity2pointcloud/lidar_camera_calib/2_7/img/4.png")
        img2 = cv2.imread("D:/fisheye+disparity2pointcloud/lidar_camera_calib/2_7/img/5.png")
        img3 = cv2.imread("D:/fisheye+disparity2pointcloud/lidar_camera_calib/2_7/img/6.png")
        # 输入三组点云的路径

        project_p(pcd1_path,pcd2_path,pcd3_path,img1,img2,img3, rr, tt, "1","2","3")
def color_img(distance):
    inc = 6.0 / 10;
    x = distance * inc;
    r = 0.0;
    g = 0.0;
    b = 0.0
    if ((0 <= x and x <= 1) or (5 <= x and x <= 6)):
        r = 1.0
    elif (4 <= x and x <= 5):
        r = x - 4
    elif (1 <= x and x <= 2):
        r = 1.0 - (x - 1)
    if (1 <= x and x <= 3):
        g = 1.0
    elif (0 <= x and x <= 1):
        g = x - 0
    elif (3 <= x and x <= 4):
        g = 1.0 - (x - 3)
    if (3 <= x and x <= 5):
        b = 1.0
    elif (2 <= x and x <= 3):
        b = x - 2;
    elif (5 <= x and x <= 6):
        b = 1.0 - (x - 5)
    r *= 255.0
    g *= 255.0
    b *= 255.0
    return r,g,b
def finetune(flag):
    if (flag):
        app = QApplication(sys.argv)
        window = DemoDoubleSpinBox()
        window.show()
        sys.exit(app.exec())

def project_p(path1,path2,path3, img1,img2,img3, rvecs, tvecs,num1,num2,num3):
    """

    根据上一步计算出来的结果，把点云投影到图像

    """

    camera_matrix = np.array([[323.6492296042108, 0., 555.7959174319013],
                              [0., 323.5287974917168, 559.7227279061037],
                              [0., 0., 1.]], dtype=np.float64
                             )
    dist_coeffs = np.array([[0],[0],[0],[0]],dtype=np.float64)

    print("->正在加载点云... ")
    point_cloud1 = o3d.io.read_point_cloud(path1)
    point_cloud2 = o3d.io.read_point_cloud(path2)
    point_cloud3 = o3d.io.read_point_cloud(path3)
    print(1)
    # point_cloud = point_cloud.uniform_down_sample(6)
    pc_as_np1 = np.asarray(point_cloud1.points)
    pc_as_np2 = np.asarray(point_cloud2.points)
    pc_as_np3 = np.asarray(point_cloud3.points)
    print(2)
    imgpoints1, tt = cv2.projectPoints(pc_as_np1, rvecs, tvecs, camera_matrix, dist_coeffs)
    imgpoints2, tt = cv2.projectPoints(pc_as_np2, rvecs, tvecs, camera_matrix, dist_coeffs)
    imgpoints3, tt = cv2.projectPoints(pc_as_np3, rvecs, tvecs, camera_matrix, dist_coeffs)
    print(3)
    i = 0

    for e in imgpoints1:
        # todo 为什么这个对显示效果影响这么大？？？
        #if int(e[0][0]) > img1.shape[1] or int(e[0][1]) > img1.shape[0] or int(e[0][0])<0 or int(e[0][1])<0:
            #continue
        distance = math.sqrt(
            pc_as_np1[i][0] * pc_as_np1[i][0] + pc_as_np1[i][1] * pc_as_np1[i][1] + pc_as_np1[i][2] * pc_as_np1[i][2])
        r1,g1,b1=color_img(distance)
        img1 = cv2.circle(img1, center=(int(e[0][0]), int(e[0][1])),
                         radius=1,
                         # color=(205, 0, 0),
                         color=(int(r1), int(g1), int(b1)),
                         # color=(int(points[i,3]), int(points[i,3]), 0),
                         # color=(int(pc_as_np[i][0]), int(pc_as_np[i][1]), int(pc_as_np[i][2])),
                         thickness=-1)
        i=i+1

    i = 0
    print(4)
    for e in imgpoints2:
        # todo 为什么这个对显示效果影响这么大？？？
        #if int(e[0][0]) > img2.shape[1] or int(e[0][1]) > img2.shape[0] or int(e[0][0]) < 0 or int(e[0][1]) < 0:
            #continue
        distance = math.sqrt(
            pc_as_np2[i][0] * pc_as_np2[i][0] + pc_as_np2[i][1] * pc_as_np2[i][1] + pc_as_np2[i][2] * pc_as_np2[i][2])
        r2, g2, b2 = color_img(distance)
        img2 = cv2.circle(img2, center=(int(e[0][0]), int(e[0][1])),
                         radius=1,
                         # color=(205, 0, 0),
                         color=(int(r2), int(g2), int(b2)),
                         # color=(int(points[i,3]), int(points[i,3]), 0),
                         # color=(int(pc_as_np[i][0]), int(pc_as_np[i][1]), int(pc_as_np[i][2])),
                         thickness=-1)
        i = i + 1
    i = 0
    print(5)
    for e in imgpoints3:
        # todo 为什么这个对显示效果影响这么大？？？
        #if int(e[0][0]) > img3.shape[1] or int(e[0][1]) > img3.shape[0] or int(e[0][0]) < 0 or int(e[0][1]) < 0:
            #continue
        distance = math.sqrt(
            pc_as_np3[i][0] * pc_as_np3[i][0] + pc_as_np3[i][1] * pc_as_np3[i][1] + pc_as_np3[i][2] * pc_as_np3[i][2])
        r3, g3, b3 = color_img(distance)
        img3 = cv2.circle(img3, center=(int(e[0][0]), int(e[0][1])),
                         radius=1,
                         # color=(205, 0, 0),
                         color=(int(r3), int(g3), int(b3)),
                         # color=(int(points[i,3]), int(points[i,3]), 0),
                         # color=(int(pc_as_np[i][0]), int(pc_as_np[i][1]), int(pc_as_np[i][2])),
                         thickness=-1)
        i = i + 1
    print(6)
    cv2.namedWindow(num1, cv2.WINDOW_NORMAL)
    cv2.imshow(num1, img1)
    cv2.namedWindow(num2, cv2.WINDOW_NORMAL)
    cv2.imshow(num2, img2)
    cv2.namedWindow(num3, cv2.WINDOW_NORMAL)
    cv2.imshow(num3, img3)
    cv2.waitKey(0)


if __name__ == "__main__":
    # 输入三组点云的路径
    pcd1_path = "D:/fisheye+disparity2pointcloud/lidar_camera_calib/2_7/pcd/4.pcd"
    pcd2_path = "D:/fisheye+disparity2pointcloud/lidar_camera_calib/2_7/pcd/5.pcd"
    pcd3_path = "D:/fisheye+disparity2pointcloud/lidar_camera_calib/2_7/pcd/6.pcd"
    flag = bool(input("continue to fine tuning(True/False) :"))
    rx = float(0)
    ry = float(0)
    rz = float(0)
    x = float(0)
    y = float(0)
    z = float(0)
    print(str(rx) + " " + str(ry) + " " + str(rz) + " " + str(x) + " " + str(y) + " " + str(z))
    finetune(flag)