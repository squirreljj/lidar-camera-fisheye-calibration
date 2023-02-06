# coding:utf-8
import os
import cv2
import numpy as np
# import glob
import open3d as o3d
import matplotlib.pyplot as plt
import time
import sys
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import (QApplication, QMainWindow, QLabel, QDoubleSpinBox)
np.set_printoptions(suppress=True)
import math

import click
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
        #print(str(rx) + " " + str(ry) + " " + str(rz) + " " + str(x) + " " + str(y) + " " + str(z))
        rr = np.array([[rx], [ry], [rz]])
        tt = np.array([[x], [y], [z]])
        img_path = img_dir + f1[5]
        test_lidar_path = pcd_dir + f2[5]
        # test_img = cv2.imread(img_path)
        test_img = cv2.imread("D:/fisheye+disparity2pointcloud/lidar_camera_calib/2_3/test.png")
        project_p_kb4(test_lidar_path, test_img, rr, tt, f1[5])
    def prefixSpinBoxTextChanged(self, a0):
        self.label_value.setText('current text is:' + str(a0))
        global ry
        ry = a0
        #print(str(rx) + " " + str(ry) + " " + str(rz) + " " + str(x) + " " + str(y) + " " + str(z))
        rr = np.array([[rx], [ry], [rz]])
        tt = np.array([[x], [y], [z]])
        img_path = img_dir + f1[5]
        test_lidar_path = pcd_dir + f2[5]
        # test_img = cv2.imread(img_path)
        test_img = cv2.imread("D:/fisheye+disparity2pointcloud/lidar_camera_calib/2_3/test.png")
        project_p_kb4(test_lidar_path, test_img, rr, tt, f1[5])
    def suffixSpinBoxTextChanged(self, a0):
        self.label_value.setText('current text is:' + str(a0))
        global rz
        rz = a0
        #print(str(rx) + " " + str(ry) + " " + str(rz) + " " + str(x) + " " + str(y) + " " + str(z))
        rr = np.array([[rx], [ry], [rz]])
        tt = np.array([[x], [y], [z]])
        img_path = img_dir + f1[5]
        test_lidar_path = pcd_dir + f2[5]
        # test_img = cv2.imread(img_path)
        test_img = cv2.imread("D:/fisheye+disparity2pointcloud/lidar_camera_calib/2_3/test.png")
        project_p_kb4(test_lidar_path, test_img, rr, tt, f1[5])
    def xSpinBoxValueChanged(self, a0):
        self.label_value.setText('current value is:' + str(a0))
        global x
        x = a0
        #print(str(rx) + " " + str(ry) + " " + str(rz) + " " + str(x) + " " + str(y) + " " + str(z))
        rr = np.array([[rx], [ry], [rz]])
        tt = np.array([[x], [y], [z]])
        img_path = img_dir + f1[5]
        test_lidar_path = pcd_dir + f2[5]
        # test_img = cv2.imread(img_path)
        test_img = cv2.imread("D:/fisheye+disparity2pointcloud/lidar_camera_calib/2_3/test.png")
        project_p_kb4(test_lidar_path, test_img, rr, tt, f1[5])
    def ySpinBoxTextChanged(self, a0):
        self.label_value.setText('current text is:' + str(a0))
        global y
        y = a0
        #print(str(rx) + " " + str(ry) + " " + str(rz) + " " + str(x) + " " + str(y) + " " + str(z))
        rr = np.array([[rx], [ry], [rz]])
        tt = np.array([[x], [y], [z]])
        img_path = img_dir + f1[5]
        test_lidar_path = pcd_dir + f2[5]
        # test_img = cv2.imread(img_path)
        test_img = cv2.imread("D:/fisheye+disparity2pointcloud/lidar_camera_calib/2_3/test.png")
        project_p_kb4(test_lidar_path, test_img, rr, tt, f1[5])
    def zSpinBoxTextChanged(self, a0):
        self.label_value.setText('current text is:' + str(a0))
        global z
        z = a0
        #print(str(rx) + " " + str(ry) + " " + str(rz) + " " + str(x) + " " + str(y) + " " + str(z))
        rr = np.array([[rx], [ry], [rz]])
        tt = np.array([[x], [y], [z]])
        img_path = img_dir + f1[5]
        test_lidar_path = pcd_dir + f2[5]
        # test_img = cv2.imread(img_path)
        test_img = cv2.imread("D:/fisheye+disparity2pointcloud/lidar_camera_calib/2_3/test.png")
        project_p_kb4(test_lidar_path, test_img, rr, tt, f1[5])

def finetune(flag):
    if (flag):
        app = QApplication(sys.argv)
        window = DemoDoubleSpinBox()
        window.show()
        sys.exit(app.exec())
def calib():
    # !!!!这里要把参数替换为你自己的相机的内参矩阵

    camera_matrix = np.array([[323.6492296042108, 0., 555.7959174319013],
                              [0., 323.5287974917168, 559.7227279061037],
                              [0., 0., 1.]], dtype=np.float64
                             )
    #相机畸变参数
    dist_coeffs = np.array([[0],[0],[0],[0]], dtype=np.float64)

    points3 = np.array(lidar_points, dtype=np.float64)
    points2 = np.array(img_points, dtype=np.float64)
    print("Camera Matrix :\n {0}".format(camera_matrix))

    (success, rotation_vector, translation_vector) = cv2.solvePnP(points3, points2, camera_matrix, dist_coeffs,
                                                                  flags=cv2.SOLVEPNP_EPNP)
    # (success, rotation_vector, translation_vector) = cv2.solvePnP(points3, points2, camera_matrix,dist_coeffs)

    print("Rotation Vector:\n {0}".format(rotation_vector))
    print("Translation Vector:\n {0}".format(translation_vector))

    rotM = cv2.Rodrigues(rotation_vector)[0]
    position = -np.matrix(rotM).T * np.matrix(translation_vector)
    print(rotM)
    #print(rotM.T)
    #print(position)
    # print("file Vector:\n {0}".format(-np.matrix(rotM).T * np.matrix(translation_vector)))

    return rotation_vector, translation_vector

def project_p_kb4(path, img, rvecs, tvecs,name):
    point_cloud = o3d.io.read_point_cloud(path)
    T = np.eye(4)
    print(str(rvecs[0]) + " " + str(rvecs[1]) + " " + str(rvecs[2]) + " " + str(tvecs[0]) + " " + str(tvecs[1]) + " " + str(tvecs[2]))
    rotation_vector = np.array([[rvecs[0]], [rvecs[1]], [rvecs[2]]])
    rotM = cv2.Rodrigues(rotation_vector)[0]
    T[:3, :3] = rotM
    T[0, 3] = tvecs[0]
    T[1, 3] = tvecs[1]
    T[2, 3] = tvecs[2]

    # 将全局点云转到雷达坐标系
    point_cloud = point_cloud.transform(T)
    points = np.array(point_cloud.points)  # 转为矩阵
    for i in points:
        #print(i[0])
        #print(math.sqrt(i[0]*i[0]+i[1]*i[1]+i[2]*i[2]))
        theta = math.acos(i[2] / math.sqrt(i[0]*i[0]+i[1]*i[1]+i[2]*i[2]))

        phi = math.atan2(i[1], i[0]);

        k2 = -0.017815017122891635;
        k3 = 0.004393633105569032;
        k4 = -0.003294336117104848;
        k5 = 0.0003341737432437223;
        p_u = float(theta +k2 * theta * theta * theta +k3 * theta * theta * theta * theta * theta +k4 * theta * theta * theta * theta * theta * theta * theta +k5 * theta * theta * theta * theta * theta * theta * theta * theta * theta)*math.cos(phi)
        p_v = float(theta + k2 * theta * theta * theta + k3 * theta * theta * theta * theta * theta + k4 * theta * theta * theta * theta * theta * theta * theta + k5 * theta * theta * theta * theta * theta * theta * theta * theta * theta) * math.sin(phi)
        u=float(324.26866659048076 * p_u + 556.6442852135398)
        v=float(323.68590505444868* p_v +  561.926198367122)
        if int(u) > img.shape[1] or int(v) > img.shape[0] or int(u)<0 or int(v)<0:
            continue
        distance = float(math.sqrt(i[0] * i[0] + i[1] * i[1] + i[2] * i[2]))
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
        img = cv2.circle(img, center=(int(u), int(v)),
                         radius=1,
                         # color=(205, 0, 0),
                         color=(int(r), int(g), int(b)),
                         # color=(int(points[i,3]), int(points[i,3]), 0),
                         # color=(int(pc_as_np[i][0]), int(pc_as_np[i][1]), int(pc_as_np[i][2])),
                         thickness=-1)
    cv2.imshow('line0', img)
    cv2.waitKey(0)




if __name__ == "__main__":
    img_dir="D:/fisheye+disparity2pointcloud/lidar_camera_calib/2_3/img/"
    res_dir="D:/fisheye+disparity2pointcloud/lidar_camera_calib/2_3/result/"
    pcd_dir="D:/fisheye+disparity2pointcloud/lidar_camera_calib/2_3/pcd/"
    crop_pcd_dir="D:/fisheye+disparity2pointcloud/lidar_camera_calib/2_3/"
    f1 = os.listdir(img_dir)
    f1.sort()
    f2 = os.listdir(pcd_dir)
    f2.sort()
    cropped_flag = False # set True to edit the pointcloud,and please save the cropped point cloud as .pcd format!
    for i in range(len(f1)):
        img_path = img_dir + f1[i]
        pcd_path = pcd_dir + f2[i]
        img=cv2.imread(img_path)
        # 主要功能是截取点云
        if cropped_flag == True:
            print("1) Press 'Y' twice to align geometry with negative direction of y-axis")
            print("2) Press 'K' to lock screen and to switch to selection mode")
            print("3) Drag for rectangle selection,")
            print("   or use ctrl + left click for polygon selection")
            print("4) Press 'C' to get a selected geometry")
            print("5) Press 'S' to save the selected geometry")
            print("6) Press 'F' to switch to freeview mode")
            pcd = o3d.io.read_point_cloud(pcd_path)
            o3d.visualization.draw_geometries_with_editing([pcd])
        #加载截取后的点云
        lidar_path = crop_pcd_dir+"crop" + str(i+1) + ".pcd"
        print("->正在加载点云... ")
        point_cloud = o3d.io.read_point_cloud(lidar_path)
        pc_as_np = np.asarray(point_cloud.points)
        # 对截取后的点云进行平面拟合
        plane_model, inliers = point_cloud.segment_plane(distance_threshold=0.005,
                                                         ransac_n=5,
                                                         num_iterations=50)
        [a, b, c, d] = plane_model
        #print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
        inlier_cloud = point_cloud.select_by_index(inliers)
        center= o3d.geometry.PointCloud()
        cen_cor = inlier_cloud.get_center()#center points
        center.points = o3d.utility.Vector3dVector([cen_cor])
        center.paint_uniform_color([0, 0, 0])
        o3d.visualization.draw_geometries([inlier_cloud,center], str(f2[i]), 1080, 720, 400, 200)
        # 对截取后的点云选取点云中心点
        # 对截取后的点云选取4个角点
        # 提取图片中标定板的角点
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, (10, 9), None)
        print('The coordinates of the four corners of the picture：')
        #print(corners[0],corners[9],corners[80],corners[89])
        #print('img center coordinates：')
        corner = (corners[0]+corners[9]+corners[80]+corners[89]) / 4
        print(corner)
        # 添加点云标定板中心点坐标
        lidar_points.append(inlier_cloud.get_center())
        print("lidar center coordinates: ", lidar_points)
        # 添加图像标定板中心点坐标
        img_points.append(corner)
        #在图片中显示中心点
        #cv2.circle(img, (int(corner[0][0]),int(corner[0][1])), 1, (0, 0, 255), thickness=-1)
        #cv2.imshow("image", img)
        #print("img_points ", img_points)

        if ret == True:
            cv2.drawChessboardCorners(img, (10, 9), corners, ret)
            plt.figure(str(f1[i]))
            #plt.imshow(img)
            #plt.show()
        else:
            print('未找到角点')
        # 提取图片中标定板的中心点
    rr, tt = calib()
    '''for i in range(len(f1)):
        img_path = img_dir + f1[i]
        test_lidar_path = pcd_dir + f2[i]
        test_img = cv2.imread(img_path)
        project_p(test_lidar_path, test_img, rr, tt, f1[i])'''

    flag = bool(input("continue to fine tuning(True/False) :"))
    rx=float(rr[0])
    ry=float(rr[1])
    rz=float(rr[2])
    x=float(tt[0])
    y=float(tt[1])
    z=float(tt[2])
    print(str(rx) + " " + str(ry) + " " + str(rz) + " " + str(x) + " " + str(y) + " " + str(z))
    finetune(flag)



