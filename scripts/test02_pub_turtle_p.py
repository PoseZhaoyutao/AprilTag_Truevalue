#! /usr/bin/env python
# @Author  : Zhao Yutao
# @Time    : 2024/7/1 12:29
# @Function: 测试redis服务端发送图像
# @mails: zhaoyutao22@mails.ucas.ac.cn
import rospy
from turtlesim.msg import Pose
import tf2_ros
import tf
from geometry_msgs.msg import TransformStamped
import sys

turtle_name = ""
def doPose(pose):
    #     1、创建发布坐标系相对关系的对象
    pub = tf2_ros.TransformBroadcaster()
    #     2、将pose转换成坐标系相对关系消息
    msg = tf2_ros.TransformStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "world"
    msg.child_frame_id = turtle_name

    # 子级坐标系相对于父级坐标系的偏移量
    msg.transform.translation.x = pose.x
    msg.transform.translation.y = pose.y
    msg.transform.translation.z = 0
    # 先从欧拉角转换四元数
    """
        乌龟是2D的，不存在x上的翻滚，Y上的俯仰，只有Z上的偏航
        pose.theta
    """
    qtn = tf.transformations.quaternion_from_euler(0, 0, pose.theta)
    msg.transform.rotation.x = qtn[0]
    msg.transform.rotation.y = qtn[1]
    msg.transform.rotation.z = qtn[2]
    msg.transform.rotation.w = qtn[3]
    pub.sendTransform(msg)


if __name__ == '__main__':
    rospy.init_node('dynamic_pub_p')
    # 解析传入参数（文件全路径+传入的args参数+节点名称+日志文件路径）
    if  len(sys.argv)!=4:
        rospy.loginfo("传入参数不对")
        sys.exit(1)
    else:
        turtle_name = sys.argv[1]
    sub = rospy.Subscriber(turtle_name+"/pose", Pose, doPose, queue_size=100)
    rospy.spin()