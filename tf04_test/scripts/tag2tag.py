# #!/usr/bin/env python
# @Author  : Zhao Yutao
# @Time    : 2024/7/1 12:29
# @Function: 测试redis服务端发送图像
# @mails: zhaoyutao22@mails.ucas.ac.cn
# """
#    解决：计算tag与tag之间距离进行x1+x2+x3+...xn（y同）计算，然后利用最小二乘法计算这些相加距离，计算出end（tagn）相对于 start（tag1）距离
#     需求:
#         求解两个tag的相对关系，然后将相对关系的 translation 与 rotation 数值转到 sub_tag 程序使用，以此确定 tagi+1 对 tagi 的位置
#         现有坐标系统，父级坐标系统 zed,下有两子级系统 tagi，tagi+1，
#         tagi 相对于 zed，以及 tagi+1相对于 zed 的关系是已知的，
#         求 tagi 与 tagi+1中的坐标关系=>△x, △y, △z即可
#         数组最后输出为end（tagn）到keng（tagn+1）的位置信息
#     实现流程:
#         1.导包
#         2.初始化 ROS 节点
#         3.创建 TF 订阅对象
#         4.1：调用 API 求出 tagi+1相对于 tagi的坐标关系=>△x, △y, △z
#         4.2：(n-2)△x, △y, △z相加为end（tagn）相对于start（tag1）的坐标信息
#         4.3：已知end相对于start，相加dx[n-1]、dy[n-1]、dz[n-1],则为keng相对于start的坐标信息
#         5.spin
# """
# # 1.导包
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf2_geometry_msgs import PointStamped
import tf
from apriltag_ros.msg import AprilTagDetectionArray
count = 1
def doPose(apriltagdetectionarray):
    try:
            global count
            if len(apriltagdetectionarray.detections) == 3:
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
                rospy.loginfo("xxxxxxx:%.5f",position0.x-position.x)
                rospy.loginfo("yyyyyyy:%.5f",position0.y-position.y)
                rospy.loginfo("zzzzzzz:%.5f",position0.z-position.z)
    except Exception as e:
        rospy.logwarn("++++++++++当前图像中并无tag信息，请调整摄像头！！！")
        rospy.logerr(e)


if __name__ == "__main__":
      # 2.初始化 ROS 节点
    rospy.init_node("sub_tf")
    sub = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, doPose)
    rospy.spin()