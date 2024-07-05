#! /usr/bin/env python
# @Author  : Zhao Yutao
# @Time    : 2024/7/1 12:29
# @Function: 测试redis服务端发送图像
# @mails: zhaoyutao22@mails.ucas.ac.cn
import math

import rospy
import tf
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import TransformStamped
from tf2_geometry_msgs import PointStamped
import tf2_ros
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from image_geometry import cameramodels
import numpy as np
#将四元数变换为4x4旋转矩阵
def quaternion_to_rotation_matrix(quat):
    q = quat.copy()
    n = np.dot(q, q)
    if n < np.finfo(q.dtype).eps:
        return np.identity(4)
    q = q * np.sqrt(2.0 / n)
    q = np.outer(q, q)
    rot_matrix = np.array(
        [[1.0 - q[2, 2] - q[3, 3], q[1, 2] + q[3, 0], q[1, 3] - q[2, 0], 0.0],
         [q[1, 2] - q[3, 0], 1.0 - q[1, 1] - q[3, 3], q[2, 3] + q[1, 0], 0.0],
         [q[1, 3] + q[2, 0], q[2, 3] - q[1, 0], 1.0 - q[1, 1] - q[2, 2], 0.0],
         [0.0, 0.0, 0.0, 1.0]],
        dtype=q.dtype)
    return rot_matrix


def align(model, data):
    """
    Input:
    model -- first trajectory (3xn) 相机坐标系下的n个点坐标
    data -- second trajectory (3xn) 世界坐标系下的n个点坐标

    Output:
    rot -- rotation matrix (3x3) 旋转矩阵
    trans -- translation vector (3x1) 平移矢量
    trans_error -- translational error per point (1xn) 将相机坐标系下的点转换到世界坐标系后与真值的均方根误差（两点距离）
    """
    np.set_printoptions(precision=3, suppress=True)
    model_zerocentered = model - model.mean(1)
    data_zerocentered = data - data.mean(1)

    W = np.zeros((3, 3))
    for column in range(model.shape[1]):
        # for column in range(3):
        W += np.outer(model_zerocentered[:, column], data_zerocentered[:, column])
    U, d, Vh = np.linalg.linalg.svd(W.transpose())
    S = np.matrix(np.identity(3))
    if (np.linalg.det(U) * np.linalg.det(Vh) < 0):
        S[2, 2] = -1
    rot = U * S * Vh
    trans = data.mean(1) - rot * model.mean(1)

    model_aligned = rot * model + trans
    alignment_error = model_aligned - data
    trans_error = np.sqrt(np.sum(np.multiply(alignment_error, alignment_error), 0)).A[0]
    return rot, trans, trans_error

def doATD(apriltagdetectionarray):
    try:
        global count
        if len(apriltagdetectionarray.detections) == 3 :
            tagid0 = apriltagdetectionarray.detections[0].id
            position0 = apriltagdetectionarray.detections[0].pose.pose.pose.position
            orientation0 = apriltagdetectionarray.detections[0].pose.pose.pose.orientation
            tagid = apriltagdetectionarray.detections[1].id
            position = apriltagdetectionarray.detections[1].pose.pose.pose.position
            orientation = apriltagdetectionarray.detections[1].pose.pose.pose.orientation
            (r0, p0, y0) = tf.transformations.euler_from_quaternion(
                [orientation0.x, orientation0.y, orientation0.z, orientation0.w])
            (r, p, y) = tf.transformations.euler_from_quaternion(
                [orientation.x, orientation.y, orientation.z, orientation.w])
            # rospy.loginfo("xxxxxxx:%.5f",position0.x-position.x)
            # rospy.loginfo("yyyyyyy:%.5f",position0.y-position.y)
            # rospy.loginfo("zzzzzzz:%.5f",position0.z-position.z)
            if tagid0[0] > tagid[0] :
                rospy.loginfo("-------:%.5f", r0 * 180 / math.pi)
                rospy.loginfo("rrrrrrrr:%.5f", (r0 - r) * 180 / math.pi)
                rospy.loginfo("pppppppp:%.5f", (p0 - p) * 180 / math.pi)
                rospy.loginfo("yyyyyyyy:%.5f", (y0 - y) * 180 / math.pi)
            else:
                rospy.loginfo("-------:%.5f", r * 180 / math.pi)

                rospy.loginfo("rrrrrrrr:%.5f", (r - r0) * 180 / math.pi)
                rospy.loginfo("pppppppp:%.5f", (p - p0) * 180 / math.pi)
                rospy.loginfo("yyyyyyyy:%.5f", (y - y0) * 180 / math.pi)

                    # rospy.loginfo("rrrrrrrr:%.5f", (r - r0) * 180 / math.pi)
                    # rospy.loginfo("pppppppp:%.5f", (p - p0) * 180 / math.pi)
                    # rospy.loginfo("yyyyyyyy:%.5f", (y - y0) * 180 / math.pi)
        # rospy.loginfo(orientation)
        # quaternion = np.asarray([orientation.w, orientation.x, orientation.y, orientation.z])
        # translation = np.asarray([position.x, position.y, position.z])
        # qua2rota = quaternion_to_rotation_matrix(quaternion)  # 四元素转4x4旋转矩阵
        # T_qua2rota = qua2rota
        # for i in range(3):
        #     T_qua2rota[i][3] = T_qua2rota[i][3] + translation[i]  # 在第四列加平移矢量
        # invT_qua2rota = np.linalg.inv(T_qua2rota)  # 矩阵求逆
        #
        # camintag_position = np.asarray([0, 0, 0, 1])  # 转为齐次坐标
        # tagincam_position = np.dot(invT_qua2rota, camintag_position)  # 两个矩阵相乘
        # rospy.loginfo(tagid)
        # rospy.loginfo(tagincam_position)
        # e = tf.transformations.euler_from_quaternion(0, 0, 0, 1)
        # rospy.loginfo(e)
    except Exception as e:
        rospy.logwarn("提示：%s", "当前图像中不包含tag------请调整摄像头位姿")

if __name__ == '__main__':
    # 2.初始化 ROS 节点
    rospy.init_node("sub_tag2zed_pose")
    sub = rospy.Subscriber("/tag_detections",AprilTagDetectionArray,doATD)
    # 6.spin
    rospy.spin()
