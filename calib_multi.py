# coding:utf-8
import os
import cv2
import numpy as np
# import glob
import open3d as o3d
import matplotlib.pyplot as plt
import time
np.set_printoptions(suppress=True)

img_points = []
lidar_points = []

def calib():
    # !!!!这里要把参数替换为你自己的相机的内参矩阵

    camera_matrix = np.array([[323.6492296042108, 0., 555.7959174319013],
                              [0., 323.5287974917168, 559.7227279061037],
                              [0., 0., 1.]], dtype=np.float64
                             )
    #相机畸变参数
    dist_coeffs = np.array([[0],[0],[-0.003294336117104848],[0.0003341737432437223],[0],[0]])

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
    print(rotM.T)
    print(position)
    # print("file Vector:\n {0}".format(-np.matrix(rotM).T * np.matrix(translation_vector)))

    return rotation_vector, translation_vector

def project_p(path, img, rvecs, tvecs):
    """

    根据上一步计算出来的结果，把点云投影到图像

    """

    camera_matrix = np.array([[323.6492296042108, 0., 555.7959174319013],
                              [0., 323.5287974917168, 559.7227279061037],
                              [0., 0., 1.]], dtype=np.float64
                             )
    dist_coeffs = np.array([[0],[0],[0],[0]])

    print("->正在加载点云... ")
    point_cloud = o3d.io.read_point_cloud(path)
    # point_cloud = point_cloud.uniform_down_sample(6)
    print(point_cloud)
    pc_as_np = np.asarray(point_cloud.points)
    #pc_as_np = pc_as_np[pc_as_np[:, 2] > 0.]

    print(pc_as_np.shape)
    print(pc_as_np.dtype)



    imgpoints2, tt = cv2.projectPoints(pc_as_np, rvecs, tvecs, camera_matrix, dist_coeffs)

    i = 0
    for e in imgpoints2:
        # if i%2 == 0:
        #     i += 1
        #     continue

        
        if int(e[0][0]) > img.shape[1] or int(e[0][1]) > img.shape[0] or int(e[0][0])<0 or int(e[0][1])<0:
            continue

        # print(int(e[0][0]), int(e[0][1]),int(points[i,3]))
        img = cv2.circle(img, center=(int(e[0][0]), int(e[0][1])),
                         radius=1,color=(205, 0, 0),
                         # color=(int(points[i,3]), 255-int(points[i,3]), 255-int(points[i,3])),
                         # color=(int(points[i,3]), int(points[i,3]), 0),
                         #color=(int(points[i, 3]), int(points[i, 3]), 0),
                         thickness=2)

        i += 1

    cv2.imshow('line0', img)
    cv2.waitKey(0)
    cv2.imwrite('calib.png', img)


if __name__ == "__main__":
    f1 = os.listdir("D:/fisheye+disparity2pointcloud/calib_data/img/")
    f1.sort()
    f2 = os.listdir("D:/fisheye+disparity2pointcloud/calib_data/pcd/")
    f2.sort()
    cropped_flag=False
    num=1        #adjust the time you need to crop the pointcloud
    #set True to edit the pointcloud,and please save the cropped point cloud as .pcd format!
    for i in range(len(f1)):
        img_path = "D:/fisheye+disparity2pointcloud/calib_data/img/" + f1[i]
        pcd_path = "D:/fisheye+disparity2pointcloud/calib_data/pcd/" + f2[i]
        img=cv2.imread(img_path)
        # 主要功能是截取点云
        if cropped_flag==True:
            for a in range(num):
                pcd = o3d.io.read_point_cloud(pcd_path)#if multi crop is chosen,save pointcloud as origin name  before the last cut out ,save it as crop1.pcd/crop2.pcd... at the last cut out 
                o3d.visualization.draw_geometries_with_editing([pcd])
        #加载截取后的点云
        lidar_path = r"D:\fisheye+disparity2pointcloud\crop" + str(i+1) + ".pcd"
        print("->正在加载点云... ")
        point_cloud = o3d.io.read_point_cloud(lidar_path)
        pc_as_np = np.asarray(point_cloud.points)
        time.sleep(5)
        # 对截取后的点云进行平面拟合
        plane_model, inliers = point_cloud.segment_plane(distance_threshold=0.005,
                                                         ransac_n=5,
                                                         num_iterations=50)
        [a, b, c, d] = plane_model
        print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
        inlier_cloud = point_cloud.select_by_index(inliers)
        inlier_cloud.paint_uniform_color([1.0, 0, 0])
        o3d.visualization.draw_geometries([inlier_cloud])
        # 对截取后的点云选取点云中心点
        print(inlier_cloud.get_center())
        # 对截取后的点云选取4个角点
        # 提取图片中标定板的角点
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, (10, 9), None)
        print('四个角点坐标分别为：')
        print(corners[0])
        print(corners[9])
        print(corners[80])
        print(corners[89])
        print('中心坐标分别为：')
        corner = (corners[0] + corners[9] + corners[80] + corners[89]) / 4
        print(corner)
        # 添加点云标定板中心点坐标
        lidar_points.append(inlier_cloud.get_center())
        print("lidar_points ", lidar_points)
        # 添加图像标定板中心点坐标
        img_points.append(corner)
        print("img_points ", img_points)

        if ret == True:
            cv2.drawChessboardCorners(img, (10, 9), corners, ret)
            plt.imshow(img)
            plt.show()
        else:
            print('未找到角点')
        # 提取图片中标定板的中心点
    rr, tt = calib()
    test_lidar_path = r"D:\fisheye+disparity2pointcloud\calib_data\pcd\fishe1.pcd"
    test_img=cv2.imread(r"D:\fisheye+disparity2pointcloud\calib_data\img\fisheye1.jpg")
    project_p(lidar_path, test_img, rr, tt)

