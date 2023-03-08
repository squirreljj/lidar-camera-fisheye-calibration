import rospy
import rosbag
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
import numpy as np

# 已知外参矩阵
T_lidar_to_cam = np.array([[1, 0, 0, 0],
                           [0, 1, 0, 0],
                           [0, 0, 1, 0],
                           [0, 0, 0, 1]])

# 创建ROS节点
rospy.init_node('update_pointcloud_color', anonymous=True)

# 创建消息桥接对象
bridge = CvBridge()

# 打开rosbag文件
bag = rosbag.Bag('/path/to/bagfile.bag', 'r')

# 创建新的rosbag文件
out_bag = rosbag.Bag('/path/to/new_bagfile.bag', 'w')

# 遍历所有消息
for topic, msg, t in bag.read_messages():

    # 如果是点云消息
    if topic == '/lidar_topic':

        # 将点云消息转换为numpy数组
        pc = np.array(list(pc2.read_points(msg)))

        # 将点云从lidar坐标系转换到camera坐标系
        pc_cam = np.dot(pc[:, :3], T_lidar_to_cam[:3, :3].T) + T_lidar_to_cam[:3, 3]

        # 从时间戳获取图像消息
        img_topic = '/image_topic'
        _, img_msg, _ = next(bag.read_messages(topics=[img_topic], start_time=msg.header.stamp, end_time=msg.header.stamp+rospy.Duration(0.1)))

        # 将图像消息转换为OpenCV格式
        img = bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')

        # 将图像像素颜色赋值给点云的RGB属性
        colors = img[np.round(pc_cam[:, 1]).astype(int), np.round(pc_cam[:, 0]).astype(int), :]
        pc[:, 3] = np.array([rgb[0]<<16 | rgb[1]<<8 | rgb[2] for rgb in colors], dtype=np.float32)

        # 创建新的点云消息
        new_pc_msg = PointCloud2()
        new_pc_msg.header = msg.header
        new_pc_msg.height = 1
        new_pc_msg.width = len(pc)
        new_pc_msg.fields = msg.fields
        new_pc_msg.is_bigendian = False
        new_pc_msg.point_step = msg.point_step
        new_pc_msg.row_step = msg.row_step
        new_pc_msg.is_dense = False
        new_pc_msg.data = pc.tostring()

        # 写入新的点云消息
        out_bag.write('/lidar_topic', new_pc_msg, t)

    # 如果不是点云消息，则直接写入新的rosbag文件中
    else:
        out_bag.write(topic, msg, t)

# 关闭rosbag文件
bag.close()
out_bag.close()
