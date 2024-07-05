#!/usr/bin/env python
# @Author  : Zhao Yutao
# @Time    : 2024/7/1 12:29
# @Function: 测试redis服务端发送图像
# @mails: zhaoyutao22@mails.ucas.ac.cn
"""
   解决：小车相对于起点坐标系tag1的坐标
    需求:
        求解tag1与小车的相对位置，利用transform以此确定小车对tag1的位置
    实现流程:
        1.导包
        2.初始化 ROS 节点
        3.创建 TF 订阅对象
        4.调用 API 求出 car(zed) 相对于 tag1 的坐标关系
        6.spin

"""
# 1.导包
import os
import numpy as np
import rospy
import tf2_ros
import tf
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

if __name__ == "__main__":
    pose = []
    # end2startPose = []
    # keng2startPose = []
    # posefile = rospy.get_param("posefile","/home/nav/demo01_ws/pose.txt")
    # f = open(posefile, 'r')
    # lines = f.readlines()
    # tag_count = len(lines) - 1

    # tag_kengid = "tag_" + str(tag_count)
    # tag_endid = "tag_" + str(tag_count - 1)
    # lines = lines[tag_count - 1:]
    # for line in lines:
    #     line = list(line.strip('\n').split(' '))
    #     pose.append(float(line[1]))
    #     pose.append(float(line[2]))
    #     pose.append(float(line[3]))
    #     pose.append(float(line[4]))
    #     pose.append(float(line[5]))
    #     pose.append(float(line[6]))
    # end2startPose = pose[:6]
    # keng2startPose = pose[-6:]
    # # 2.初始化 ROS 节点
    # 多次存储xyz信息进行高斯滤波
    dx = []
    dy = []
    dz = []
    # 多次存储rpy信息进行高斯滤波
    dr = []
    dp = []
    dyy = []
    rospy.init_node("frames_sub_p")
    # 3.创建 TF 订阅对象
    # pubkeng = rospy.Publisher("/odom_hole", Odometry, queue_size=100)
    broadcaster = tf2_ros.TransformBroadcaster()
    #00000：因为不需要订阅camera，直接转成车的，所以不需要了
    # camera_name = "camera"
    # camera_name = "zed2_left_camera_frame"
    car_name=rospy.get_param("car_name", "base_link")
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)
    rate = rospy.Rate(1)
    car2s = []
    camera_link = rospy.get_param("camera_linkname","camera_link")
    while not rospy.is_shutdown():
        try:
            # 发布tag_end相对于tag_1的位姿信息
            # msg = tf2_ros.TransformStamped()
        # 3.调用 API 求出 car 相对于 tag1 的坐标关系
        ####之前想法##### 统一世界坐标系左手系，z向上x向左y向前
        ####同上，所以更改方案##### zed相对于tag建立的坐标系是左手x向上y向左z向后，x做世界坐标系z，-y做世界坐标系x，-z做世界坐标系y
        # tag默认建立的时左手x向脸，z轴向后，y轴向右的，相对于start在static中发布了其转换成z轴向脸，Y轴向左，x轴向后的左手系，然后x变为-x，即为右手系世界坐标系
        # 世界系坐标应该是右手系，z向脸x向前y向左    zed2_left_camera_optical_frame
            #lookup_transform(self, target_frame, source_frame, time, timeout=rospy.Duration(0.0)):
            # 这里的子级坐标系就是camera2car中的父级坐标系car
            # 0000000：tf中发布了相机camera相对于小车car的位姿信息，其实可以直接转成start相对于小车car的。下面这句代码可以不用
            # camera2car = buffer.lookup_transform("car", "camera", rospy.Time(0))
            # keng = buffer.lookup_transform(tag_kengid, car_name, rospy.Time(0))
            # rospy.loginfo(keng.transform.translation.x)
            tfs = buffer.lookup_transform("start",camera_link, rospy.Time(0))
            car2s.append(tfs)
            dx.append(tfs.transform.translation.x)
            dy.append(tfs.transform.translation.y)
            dz.append(tfs.transform.translation.z)
            # print("-----------------")
            #print(tfs.transform.rotation.x, tfs.transform.rotation.y,tfs.transform.rotation.z,tfs.transform.rotation.w)
            (r, p, y) = tf.transformations.euler_from_quaternion(
            [tfs.transform.rotation.x, tfs.transform.rotation.y,tfs.transform.rotation.z,tfs.transform.rotation.w])
            dr.append(r)
            dp.append(p)
            dyy.append(y)
            # rospy.loginfo("car相对start:x=%.5f ,y=%.5f ,z=%.5f, r=%.5f, p=%.5f, y=%.5f",
            #               tfs.transform.translation.x,
            #               tfs.transform.translation.y,
            #               tfs.transform.translation.z,
            #               r*180/3.14,p*180/3.14,y*180/3.14)
            # rospy.loginfo([tfs.transform.rotation.x, tfs.transform.rotation.y,tfs.transform.rotation.z,tfs.transform.rotation.w])
            # buffer.clear()
            if len(car2s)==10:
                x_w = np.mean(dx)
                y_w = np.mean(dy)
                z_w = np.mean(dz)
                r_w = np.mean(dr)
                p_w = np.mean(dp)
                yy_w = np.mean(dyy)
                qtn = tf.transformations.quaternion_from_euler(r_w,p_w,yy_w)
                tfs_final = [x_w,y_w,z_w,qtn[0],qtn[1],qtn[2],qtn[3]]
            if len(car2s)>=10:
                # print(tfs_final)
                tfx = TransformStamped()
                tfx.header.stamp = rospy.Time.now()  #tfs.header.stamp
                tfx.header.frame_id = "start"
                tfx.child_frame_id = "left_navigation_camera_link_yt"
                tfx.transform.translation.x = tfs_final[0]
                tfx.transform.translation.y = tfs_final[1]
                tfx.transform.translation.z = tfs_final[2]
                # qtn = tf.transformations.quaternion_from_euler(-r,-p,-y)
                #qtn = tf.transformations.quaternion_from_euler(r,p,y)
                tfx.transform.rotation.x = tfs_final[3]
                tfx.transform.rotation.y = tfs_final[4]
                tfx.transform.rotation.z = tfs_final[5]
                tfx.transform.rotation.w = tfs_final[6]
                # 4-3.广播器发布数据发布相对
                broadcaster.sendTransform(tfx)


            # tfs = buffer.lookup_transform(camera_link, car_name, rospy.Time(0))
            # tfx = TransformStamped()
            # tfx.header.stamp = rospy.Time.now()
            # tfx.header.frame_id = "left_navigation_camera_link1"
            # tfx.child_frame_id = "base_link1"
            # tfx.transform.translation.x = tfs.transform.translation.x
            # tfx.transform.translation.y = tfs.transform.translation.y
            # tfx.transform.translation.z = tfs.transform.translation.z
            # tfx.transform.rotation.x = tfs.transform.rotation.x
            # tfx.transform.rotation.y = tfs.transform.rotation.y
            # tfx.transform.rotation.z = tfs.transform.rotation.z
            # tfx.transform.rotation.w = tfs.transform.rotation.w
            # # 4-3.广播器发布数据发布相对
            # broadcaster.sendTransform(tfx)

                tfs = buffer.lookup_transform("start","left_navigation_camera_link_yt",rospy.Time(0))
                # buffer.clear()
                (r, p, y) = tf.transformations.euler_from_quaternion(
                [tfs.transform.rotation.x, tfs.transform.rotation.y,tfs.transform.rotation.z,tfs.transform.rotation.w])
                rospy.loginfo("car相对start:x=%.5f ,y=%.5f ,z=%.5f, r=%.5f, p=%.5f, y=%.5f",
                            tfs.transform.translation.x,
                            tfs.transform.translation.y,
                            tfs.transform.translation.z,
                            r*180/3.14,p*180/3.14,y*180/3.14)
            #rospy.loginfo([tfs.transform.rotation.x, tfs.transform.rotation.y,tfs.transform.rotation.z,tfs.transform.rotation.w])
            # dx = tfs.transform.translation.x - (-0.1359)
            # dy = tfs.transform.translation.y - 0.0114
            # dz = tfs.transform.translation.z - 0.4214
            # rospy.logerr("x轴移动:%.5f", dx)
            # rospy.logerr("y轴移动:%.5f", dy)
            # rospy.logerr("z轴移动:%.5f", dz)
        except Exception as e:
            rospy.logerr("NONE START")
        # finally:
        #     # 切换到小车导航，keng相对于car的并且发布出去
        #     try:
        #         tfs_end = buffer.lookup_transform(tag_endid, car_name, rospy.Time(0))
        #         buffer.clear()
        #         (r, p, y) = tf.transformations.euler_from_quaternion(
        #             [tfs_end.transform.rotation.x, tfs_end.transform.rotation.y, tfs_end.transform.rotation.z, tfs_end.transform.rotation.w])
        #         car2startPose = [-tfs_end.transform.translation.y + end2startPose[0],
        #             -tfs_end.transform.translation.z + end2startPose[1],
        #             tfs_end.transform.translation.x + end2startPose[2],
        #             (-p + end2startPose[3]),
        #             (-y + end2startPose[4]),
        #             (r + end2startPose[5])]
        #         # 变为相机建系z轴向前，y轴向下，x轴向右。
        #         keng2carPose = [car2startPose[0] - keng2startPose[0],
        #                         car2startPose[1] - keng2startPose[1],
        #                         car2startPose[2] - keng2startPose[2],
        #                         car2startPose[3] - keng2startPose[3],
        #                         car2startPose[4] - keng2startPose[4],
        #                         car2startPose[5] - keng2startPose[5]
        #                         ]
        #         # 需要改成相对于base_link_start
        #         tfx = Odometry()
        #         tfx.header.stamp = rospy.Time.now()
        #         tfx.header.frame_id = "car"
        #         tfx.child_frame_id = "hole"
        #         tfx.pose.pose.position.x = keng2carPose[0]
        #         tfx.pose.pose.position.y = keng2carPose[1]
        #         tfx.pose.pose.position.z = keng2carPose[2]
        #         qtn = tf.transformations.quaternion_from_euler(keng2carPose[3], keng2carPose[4], keng2carPose[5])
        #         tfx.pose.pose.orientation.x = qtn[0]
        #         tfx.pose.pose.orientation.y = qtn[1]
        #         tfx.pose.pose.orientation.z = qtn[2]
        #         tfx.pose.pose.orientation.w = qtn[3]
        #         # 4 发布数据发布相对
        #         pubkeng.publish(tfx)
        #         rospy.loginfo("Hole相对car:x=%.5f ,y=%.5f ,z=%.5f, r=%.5f, p=%.5f, y=%.5f",
        #                   keng2carPose[0], keng2carPose[1], keng2carPose[2], keng2carPose[3], keng2carPose[4], keng2carPose[5])
        #     except Exception as e:
        #         rospy.logerr("NONE END")
        rate.sleep()
    # 6.spin
    # buffer.clear()
    rospy.spin()
