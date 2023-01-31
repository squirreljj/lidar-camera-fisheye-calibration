# coding:utf-8

import cv2
import numpy as np
# import glob
import open3d as o3d

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
    dist_coeffs = np.array([[-0.017815017122891635],[0.004393633105569032],[0],[0],[-0.003294336117104848],[0.0003341737432437223],[0],[0]])

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
    dist_coeffs = np.array([[-0.017815017122891635],[0.004393633105569032],[0],[0],[-0.003294336117104848],[0.0003341737432437223],[0],[0]])

    print("->正在加载点云... ")
    point_cloud = o3d.io.read_point_cloud(path)
    # point_cloud = point_cloud.uniform_down_sample(6)
    print(point_cloud)
    pc_as_np = np.asarray(point_cloud.points)
    # pc_as_np = pc_as_np[pc_as_np[:, 2] < 0.]

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
    img_path = r"D:\fisheye+disparity2pointcloud\fisheye1.png"
    img = cv2.imread(img_path)
    print(img.shape)

    # 需要先把LVX的文件转换为PCD的格式
    lidar_path = r"D:\fisheye+disparity2pointcloud\1673581435.184226036.pcd"
    print("->正在加载点云... ")
    point_cloud = o3d.io.read_point_cloud(lidar_path)
    print(point_cloud)
    pc_as_np = np.asarray(point_cloud.points)

    for i in range(4):
        # ！！！为了防止标定点太多，顺序混乱，一个一个点的标记;标记完要关闭图像或者点云显示画面
        get_img_points(img)
        get_lidar_points(point_cloud, pc_as_np)  # 按住shift+左键选点

    rr, tt = calib()
    project_p(lidar_path, img, rr, tt)
