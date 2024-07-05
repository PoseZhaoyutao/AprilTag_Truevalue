#!/usr/bin/env python
# @Author  : Zhao Yutao
# @Time    : 2024/7/1 12:29
# @Function: 测试redis服务端发送图像
# @mails: zhaoyutao22@mails.ucas.ac.cn
"""
   解决：小车相对于坑的坐标
    需求:
        求解car与的相对位置，提前利用Apriltag测出坑相对于end的位置
        再利用tag2tag程序中已知end相对于start坐标，解出坑相对于start的坐标（即绝对坐标）
        以此在识别end（tagn）后，最后校准car相对于start的绝对坐标
!!!!!!!!思路错误，应该在tag2tag程序中求出数组中最后一个应该是keng相对于end的位置信息
!!!!!!!!此程序段根据最后输出位置信息加上car2end求car2keng的位置
        进行求解car相对于keng的位姿，再进行自主导航
    实现流程:
        1.导包
        2.初始化 ROS 节点
        3.创建 TF 订阅对象
        4.已知坑相对于 start 的绝对坐标
        5.调用 API 求出 car 相对于 end 的坐标关系，得出car相对于start位姿
        6.求解 car相对于 keng 的位姿
        7.spin
"""
# 1.导包2
import math

import rospy
import tf2_ros
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from tf2_geometry_msgs import PointStamped
import tf
if __name__ == "__main__":

    Pose = []
    end2startPose = []
    keng2startPose = []
    # 2.初始化 ROS 节点
    rospy.init_node("frames_sub_p")
    # pub = tf2_ros.TransformBroadcaster()
    # #     2、将pose转换成坐标系相对关系消息
    # msg = tf2_ros.TransformStamped()
    # msg.header.stamp = rospy.Time.now()
    # msg.header.frame_id = "world"
    # msg.child_frame_id = turtle_name
    broadcaster = tf2_ros.TransformBroadcaster()
    pubcar = rospy.Publisher("/car2startPose",PoseStamped, queue_size=100)
    pubkeng = rospy.Publisher("/keng2startPose",PoseStamped,queue_size=100)
    # 3.创建 TF 订阅对象
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)
    rate = rospy.Rate(10)
    f = open('/pose.txt', 'r')
    lines = f.readlines()
    tag_count = len(lines)-1
    tag_kengid = "tag_"+ str(tag_count)
    tag_endid = "tag_"+ str(tag_count-1)
    lines = lines[tag_count-1:]
    for line in lines:
        line = list(line.strip('\n').split(' '))
        Pose.append(float(line[1]))
        Pose.append(float(line[2]))
        Pose.append(float(line[3]))
        Pose.append(float(line[4]))
        Pose.append(float(line[5]))
        Pose.append(float(line[6]))
    end2startPose = Pose[:6]
    keng2startPose = Pose[-6:]
    print(end2startPose)
    print(keng2startPose)
    car_name = rospy.get_param("car_name", "car")
    while not rospy.is_shutdown():
        try:
        #     先默认tag_4即keng的位置,tag_3即end的位置
        # 4.调用 API 求出 car 相对于 start(tag1) 的坐标关系
        # 统一世界坐标系左手系，z向上x向左y向前
        # zed相对于tag建立的坐标系是左手x向上y向左z向后，x做世界坐标系z，-y做世界坐标系x，-z做世界坐标系y
            #lookup_transform(self, target_frame, source_frame, time, timeout=rospy.Duration(0.0)):
            # 4.1 car对于end的相对位置
            tfs = buffer.lookup_transform(tag_endid, car_name, rospy.Time(0))
            # end、keng相对于start坐标信息
            # 求car相对于start的坐标信息
            (r, p, y)= tf.transformations.euler_from_quaternion(
            [tfs.transform.rotation.x, tfs.transform.rotation.y, tfs.transform.rotation.z,tfs.transform.rotation.w])
            # (r, p, y) = tf.transformations.euler_from_quaternion(
            # [-tfs.transform.rotation.y, -tfs.transform.rotation.z, tfs.transform.rotation.x, tfs.transform.rotation.w])
            car2startPose = [
                -tfs.transform.translation.y + end2startPose[0],
                -tfs.transform.translation.z + end2startPose[1],
                tfs.transform.translation.x + end2startPose[2],
                (-p + end2startPose[3]),
                (-y + end2startPose[4]),
                (r + end2startPose[5])
                # (r + end2startPose[3]) * 180 / math.pi,
                # (p + end2startPose[4]) * 180 / math.pi,
                # (y + end2startPose[5]) * 180 / math.pi
                            ]
            # 求car相对于keng的坐标信息
            # car2kengPose = [car2startPose[0] - keng2startPose[0],
            #                 car2startPose[1] - keng2startPose[1],
            #                 car2startPose[2] - keng2startPose[2],
            #                 car2startPose[3] - keng2startPose[3],
            #                 car2startPose[4] - keng2startPose[4],
            #                 car2startPose[5] - keng2startPose[5]
            #                 ]
            # rospy.loginfo(car2startPose)
            # msg = tf2_ros.TransformStamped()
            # msg.header.stamp = rospy.Time.now()
            # msg.header.frame_id = "tag_1"
            # msg.child_frame_id = car_name
            # msg.transform.translation.x = car2startPose[0]
            # msg.transform.translation.y = car2startPose[1]
            # msg.transform.translation.z = car2startPose[2]
            # qtn = tf.transformations.quaternion_from_euler(car2startPose[3], car2startPose[4], car2startPose[5])
            # msg.transform.rotation.x = qtn[0]
            # msg.transform.rotation.y = qtn[1]
            # msg.transform.rotation.z = qtn[2]
            # msg.transform.rotation.w = qtn[3]
            # broadcaster.sendTransform(msg)
            # msg.header.stamp = rospy.Time.now()
            # msg.header.frame_id = "tag_1"
            # msg.child_frame_id = tag_kengid
            # msg.transform.translation.x = keng2startPose[0]
            # msg.transform.translation.y = keng2startPose[1]
            # msg.transform.translation.z = keng2startPose[2]
            # qtn = tf.transformations.quaternion_from_euler(keng2startPose[3], keng2startPose[4], keng2startPose[5])
            # msg.transform.rotation.x = qtn[0]
            # msg.transform.rotation.y = qtn[1]
            # msg.transform.rotation.z = qtn[2]
            # msg.transform.rotation.w = qtn[3]
            # broadcaster.sendTransform(msg)
            # rospy.loginfo(car2kengPose)
            # 构建相对位姿对象并发布至tf
            # tfx = PoseStamped()
            # tfx.header.stamp = rospy.Time.now()
            # tfx.header.frame_id = "/start"
            # tfx.pose.position.x = car2startPose[0]
            # tfx.pose.position.y = car2startPose[1]
            # tfx.pose.position.z = car2startPose[2]
            # qtn = tf.transformations.quaternion_from_euler(car2startPose[3], car2startPose[4], car2startPose[5])
            # tfx.pose.orientation.x = qtn[0]
            # tfx.pose.orientation.y = qtn[1]
            # tfx.pose.orientation.z = qtn[2]
            # tfx.pose.orientation.w = qtn[3]
            # pubcar.publish(tfx)
            # kengpose = Pose()
            # kengpose.header.stamp = rospy.Time.now()
            # kengpose.header.frame_id = "/start"
            # kengpose.position.x = keng2startPose[0]
            # kengpose.position.y = keng2startPose[1]
            # kengpose.position.z = keng2startPose[2]
            # qtn2 = tf.transformations.quaternion_from_euler(keng2startPose[3], keng2startPose[4], keng2startPose[5])
            # kengpose.orientation.x = qtn[0]
            # kengpose.orientation.y = qtn[1]
            # kengpose.orientation.z = qtn[2]
            # kengpose.orientation.w = qtn[3]
            # pubkeng.publish(kengpose)
        except Exception as e:
            rospy.logerr("错误提示:%s", e)
        rate.sleep()
    # 6.spin
    rospy.spin()
