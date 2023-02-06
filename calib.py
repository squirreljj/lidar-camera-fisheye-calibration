# coding:utf-8
import math
import cv2
import numpy as np
# import glob
import open3d as o3d
import os
import matplotlib.pyplot as plt
np.set_printoptions(suppress=True)

img_points = []

lidar_points = []


def mouse(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        xy = "%d,%d" % (x, y)
        cv2.circle(img, (x, y), 1, (0, 0, 255), thickness=-1)
        cv2.putText(img, xy, (x, y), cv2.FONT_HERSHEY_PLAIN,
                    1.2, (0, 0, 255), thickness=1)
        img_points.append([x, y])
        print("img_points ", img_points)
        cv2.imshow("image", img)


def get_img_points(img):
    # return np.array([[483, 910], [766, 450], [1094, 487], [1200, 493], [1863, 380], [1244, 820]])

    # img = cv2.imread("2.jpg")
    # cv2.namedWindow("image", cv2.AUTOSIZE)
    cv2.namedWindow("image", cv2.WINDOW_NORMAL)  # 加上它才会显示完整图片
    cv2.imshow("image", img)
    #cv2.resizeWindow("image", img.shape[1], img.shape[0])
    cv2.setMouseCallback("image", mouse)

    cv2.waitKey(0)
    cv2.destroyAllWindows()


def get_lidar_points(point_cloud, pc_as_np):
    # return

    # point_cloud2 = point_cloud.voxel_down_sample(voxel_size=0.5)  # voxel_down_sample 把点云分配在三维的网格中取平均值

    print("   按住 [shift + right click] 选择点 ; 按住shift+鼠标右键可以实现取消选择。")
    #vis=o3d.visualization.draw_geometries_with_editing([point_cloud])
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(point_cloud)
    vis.run()  # user picks points
    vis.destroy_window()
    # print("")
    pick_p = vis.get_picked_points()
    # print(pick_p)  # 会打印出选择的点的序号
    # print(pc_as_np[pick_p])  # 会打印出选择的点的序号
    lidar_points.append(pc_as_np[pick_p])
    print("lidar_points ", lidar_points)


def calib():
    # !!!!这里要把参数替换为你自己的相机的内参矩阵

    camera_matrix = np.array([[323.6492296042108, 0., 555.7959174319013],
                              [0., 323.5287974917168, 559.7227279061037],
                              [0., 0., 1.]], dtype=np.float64
                             )
    #相机畸变参数
    dist_coeffs = np.array([[0],[0],[0],[0]],dtype=np.float64)

    points3 = np.array(lidar_points, dtype=np.float64)
    points2 = np.array(img_points, dtype=np.float64)
    print("Camera Matrix :\n {0}".format(camera_matrix))

    (success, rotation_vector, translation_vector) = cv2.solvePnP(points3, points2, camera_matrix, dist_coeffs,
                                                                  flags=cv2.SOLVEPNP_EPNP)
    points2D_reproj = cv2.projectPoints(points3, rotation_vector, translation_vector, camera_matrix, dist_coeffs)[0]
    error = (points2D_reproj - points2)
    rmse = np.sqrt(np.mean(error[:, 0] ** 2 + error[:, 1] ** 2))
    print(rmse)
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

def NumpyToPCD(xyz):
    """ convert numpy ndarray to open3D point cloud
    Args:
        xyz (ndarray):
    Returns:
        [open3d.geometry.PointCloud]:
    """

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)

    return pcd

def project_p(path, img, rvecs, tvecs):
    """

    根据上一步计算出来的结果，把点云投影到图像

    """

    camera_matrix = np.array([[323.6492296042108, 0., 555.7959174319013],
                              [0., 323.5287974917168, 559.7227279061037],
                              [0., 0., 1.]], dtype=np.float64
                             )
    dist_coeffs = np.array([[0],[0],[0],[0]],dtype=np.float64)

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
        # todo 为什么这个对显示效果影响这么大？？？
        if int(e[0][0]) > img.shape[1] or int(e[0][1]) > img.shape[0] or int(e[0][0])<0 or int(e[0][1])<0:
            continue

        # print(int(e[0][0]), int(e[0][1]),int(points[i,3]))
        distance = math.sqrt(
            pc_as_np[i][0] * pc_as_np[i][0] + pc_as_np[i][1] * pc_as_np[i][1] + pc_as_np[i][2] * pc_as_np[i][2])
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
        img = cv2.circle(img, center=(int(e[0][0]), int(e[0][1])),
                         radius=1,
                         # color=(205, 0, 0),
                         color=(int(r), int(g), int(b)),
                         # color=(int(points[i,3]), int(points[i,3]), 0),
                         # color=(int(pc_as_np[i][0]), int(pc_as_np[i][1]), int(pc_as_np[i][2])),
                         thickness=-1)
        i += 1

    cv2.imshow('line0', img)
    cv2.waitKey(0)
    cv2.imwrite('calib.png', img)


if __name__ == "__main__":
    img_dir = "D:/fisheye+disparity2pointcloud/lidar_camera_calib/2_3/img/"
    pcd_dir = "D:/fisheye+disparity2pointcloud/lidar_camera_calib/2_3/pcd/"
    f1 = os.listdir(img_dir)
    f1.sort()
    f2 = os.listdir(pcd_dir)
    f2.sort()
    cropped_flag = False  # set True to edit the pointcloud,and please save the cropped point cloud as .pcd format!
    for i in range(len(f1)):
        for j in range(2):
            img_path = img_dir + f1[i]
            pcd_path = pcd_dir + f2[i]
            img = cv2.imread(img_path)
            point_cloud = o3d.io.read_point_cloud(pcd_path)
            pc_as_np = np.asarray(point_cloud.points)
            get_img_points(img)
            get_lidar_points(point_cloud, pc_as_np)  # 按住shift+左键选点
    rr, tt = calib()
    lidar_path = r"D:\fisheye+disparity2pointcloud\lidar_camera_calib\2_3\pcd\1.pcd"
    tes_img=cv2.imread(img_dir + f1[0])
    project_p(lidar_path, tes_img, rr, tt)
    '''img_path = r"D:\fisheye+disparity2pointcloud\lidar_camera_calib\2_3\img\1.jpg"
    img = cv2.imread(img_path)
    print(img.shape)

    # 需要先把LVX的文件转换为PCD的格式
    lidar_path = r"D:\fisheye+disparity2pointcloud\lidar_camera_calib\2_3\pcd\1.pcd"
    print("->正在加载点云... ")
    point_cloud = o3d.io.read_point_cloud(lidar_path)
    pc_as_np = np.asarray(point_cloud.points)


    for i in range(6):
        # ！！！为了防止标定点太多，顺序混乱，一个一个点的标记;标记完要关闭图像或者点云显示画面
        get_img_points(img)
        get_lidar_points(point_cloud, pc_as_np)  # 按住shift+左键选点

    rr, tt = calib()
    #重投影可视化
    project_p(lidar_path, img, rr, tt)'''

