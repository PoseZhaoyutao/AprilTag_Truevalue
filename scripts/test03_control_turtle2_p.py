#! /usr/bin/env python
# @Author  : Zhao Yutao
# @Time    : 2024/7/1 12:29
# @Function: 测试redis服务端发送图像
# @mails: zhaoyutao22@mails.ucas.ac.cn
import math

import rospy
import tf2_ros
from tf2_geometry_msgs import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped, Twist

if __name__ == '__main__':
    rospy.init_node("static_sub_p")
    # 创建缓存对象
    buffer = tf2_ros.Buffer()
    # 创建订阅对象
    sub = tf2_ros.TransformListener(buffer)
    # 创建速度消息发布对象
    pub = rospy.Publisher("/turtle2/cmd_vel",Twist, queue_size=100)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            # 计算son1相对于son2的坐标关系
            """
                参数1：目标坐标系
                参数2：源坐标系
                参数3：时间戳rospy.Time(0)--取时间间隔最近的两个坐标系帧（son1相对world与son2相对world）计算结果
                返回值：son1与son2的坐标关系
            """
            ts = buffer.lookup_transform("turtle2","turtle1",rospy.Time(0))
            rospy.loginfo("父级坐标系：%s，子级坐标系：%s,偏移量：（%.2f,%.2f,%.2f）",
                          ts.header.frame_id,
                          ts.child_frame_id,
                          ts.transform.translation.x,
                          ts.transform.translation.y,
                          ts.transform.translation.z)
        #     组织Twist消息
            twist = Twist()
            # 线速度=系数*坐标系远点的间距（）=系数*（x^2+y^2）再开方
            # 角速度=系数*夹角=系数*atan2（y,x)
            twist.linear.x =5 * math.sqrt(math.pow(ts.transform.translation.x,2)+math.pow(ts.transform.translation.y,2))
            twist.angular.z = 4 * math.atan2(ts.transform.translation.y, ts.transform.translation.x)
            pub.publish(twist)
        # 发布消息控制turtle2
        except Exception as e:
            rospy.logwarn("错误提示：%s", e)
        rate.sleep()