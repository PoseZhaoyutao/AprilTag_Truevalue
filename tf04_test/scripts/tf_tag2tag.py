#! /usr/bin/env python
# @Author  : Zhao Yutao
# @Time    : 2024/7/1 12:29
# @Function: 测试redis服务端发送图像
# @mails: zhaoyutao22@mails.ucas.ac.cn
"""
   解决：计算tag与tag之间距离进行x1+x2+x3+...xn（y同）计算，然后利用最小二乘法计算这些相加距离，计算出end（tagn）相对于 start（tag1）距离
    需求:
        求解两个tag的相对关系，然后将相对关系的 translation 与 rotation 数值转到 sub_tag 程序使用，以此确定 tagi+1 对 tagi 的位置
        现有坐标系统，父级坐标系统 zed,下有两子级系统 tagi，tagi+1，
        tagi 相对于 zed，以及 tagi+1相对于 zed 的关系是已知的，
        求 tagi 与 tagi+1中的坐标关系=>△x, △y, △z即可
        数组最后输出为end（tagn）到keng（tagn+1）的位置信息
    实现流程:
        1.导包
        2.初始化 ROS 节点
        3.创建 TF 订阅对象
        4.1：调用 API 求出 tagi+1相对于 tagi的坐标关系=>△x, △y, △z
        4.2：(n-2)△x, △y, △z相加为end（tagn）相对于start（tag1）的坐标信息
        4.3：已知end相对于start，相加dx[n-1]、dy[n-1]、dz[n-1],则为keng相对于start的坐标信息
        5.spin
"""
# 1.导包
# 不要使用 geometry_msgs,需要使用 tf2 内置的消息类型
import math

from tf2_geometry_msgs import PointStamped
import rospy
import tf2_ros
import tf
from turtlesim.msg import Pose
from geometry_msgs.msg import TransformStamped
from apriltag_ros.msg import AprilTagDetectionArray
import matplotlib.pyplot as plt
import numpy as np
# from scipy.ndimage.filters import gaussian_filter

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


count = 1
tag_count = 9 # number of tag
# 多次存储xyz信息进行高斯滤波
dx = []
dy = []
dz = []
# 多次存储rpy信息进行高斯滤波
dr = []
dp = []
dyy = []
# 存储相邻tag间xyz信息
x_w = []
y_w = []
z_w = []
# 存储相邻tag间rpy信息
r_w = []
p_w = []
yy_w = []

flag = True
f = open('pose.txt', 'w')
trajfile = open('traj.txt','w')
def doPose(apriltagdetectionarray):
    try:
            global count
            if len(apriltagdetectionarray.detections) == 3 and flag:
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

                if count < tag_count:
                    if tagid0[0] > tagid[0] and tagid[0] == count:
                        dx.append(position0.x-position.x)
                        dy.append(position0.y-position.y)
                        dz.append(position0.z-position.z)
                        dr.append(r0-r)
                        dp.append(p0-p)
                        dyy.append(y0-y)
                        # rospy.loginfo("rrrrrrrr:%.5f", (r0 - r) * 180 / math.pi)
                        # rospy.loginfo("pppppppp:%.5f", (p0 - p) * 180 / math.pi)
                        # rospy.loginfo("yyyyyyyy:%.5f", (y0 - y) * 180 / math.pi)
                    elif tagid0[0] < tagid[0] and tagid0[0] == count:
                        dx.append(position.x - position0.x)
                        dy.append(position.y - position0.y)
                        dz.append(position.z - position0.z)
                        dr.append(r - r0)
                        dp.append(p - p0)
                        dyy.append(y - y0)
                        # rospy.loginfo("rrrrrrrr:%.5f", (r - r0) * 180 / math.pi)
                        # rospy.loginfo("pppppppp:%.5f", (p - p0) * 180 / math.pi)
                        # rospy.loginfo("yyyyyyyy:%.5f", (y - y0) * 180 / math.pi)
                    else:
                        rospy.logwarn("**********已计算过的tag！！！")
                else:
                    f.write("end相对于start（tag1）的位姿信息"+" "+
                            str(sum(x_w) - x_w[tag_count - 2])+" "+
                            str(sum(y_w) - y_w[tag_count - 2])+" "+
                            str(sum(z_w) - z_w[tag_count - 2])+" "+
                            str(sum(r_w) - r_w[tag_count - 2])+" "+
                            str(sum(p_w) - p_w[tag_count - 2])+" "+
                            str(sum(yy_w) - yy_w[tag_count - 2])+
                            "\n")
                    f.write("keng相对于start（tag1）的位姿信息"+" "+
                                  str(sum(x_w))+" "+ str(sum(y_w))+" "+str(sum(z_w))+" "+
                                  str(sum(r_w))+" "+ str(sum(p_w))+" "+str(sum(yy_w))+"\n")

                    f.close()
                    trajfile.close()
                    end2start = (sum(x_w) - x_w[tag_count - 2],
                                 sum(y_w) - y_w[tag_count - 2],
                                 sum(z_w) - z_w[tag_count - 2],
                                 sum(r_w) - r_w[tag_count - 2],
                                 sum(p_w) - p_w[tag_count - 2],
                                 sum(yy_w) - yy_w[tag_count - 2])
                    keng2start = (sum(x_w), sum(y_w), sum(z_w), sum(r_w)*180/math.pi, sum(p_w)*180/math.pi, sum(yy_w)*180/math.pi)
                    rospy.loginfo("end相对于start（tag1）的位姿信息：%.5f, %.5f, %.5f, %.5f, %.5f, %.5f",
                                  sum(x_w) - x_w[tag_count - 2],
                                  sum(y_w) - y_w[tag_count - 2],
                                  sum(z_w) - z_w[tag_count - 2],
                                  sum(r_w) - r_w[tag_count - 2],
                                  sum(p_w) - p_w[tag_count - 2],
                                  sum(yy_w) - yy_w[tag_count - 2])
                    rospy.loginfo("keng相对于start（tag1）的位姿信息：%.5f, %.5f, %.5f, %.5f, %.5f, %.5f",
                                  sum(x_w), sum(y_w), sum(z_w), sum(r_w)*180/math.pi, sum(p_w)*180/math.pi, sum(yy_w)*180/math.pi)
                    rospy.logwarn("---------超出tag数量范围!!!!!")
                    sub.unregister()
                    # rospy.on_shutdown("已遍历所有tag！注意查看log文件")
                if len(dx) == 100:
                    count += 1
                    rospy.loginfo(count)
                    # chazhi = 0
                    # chazhi1 = 0
                    # chazhi2 = 0
                    # blurred_x = gaussian_filter(dx, 100)
                    # blurred_y = gaussian_filter(dy, 100)
                    # blurred_z = gaussian_filter(dz, 100)
                    # blurred_r = gaussian_filter(dr, 100)
                    # blurred_p = gaussian_filter(dp, 100)
                    # blurred_yy = gaussian_filter(dyy, 100)
                    blurred_x = np.mean(dx)
                    blurred_y = np.mean(dy)
                    blurred_z = np.mean(dz)
                    blurred_r = np.mean(dr)
                    blurred_p = np.mean(dp)
                    blurred_yy = np.mean(dyy)
                    x_w.append(np.mean(blurred_x))
                    y_w.append(np.mean(blurred_y))
                    z_w.append(np.mean(blurred_z))
                    r_w.append(np.mean(blurred_r))
                    p_w.append(np.mean(blurred_p))
                    yy_w.append(np.mean(blurred_yy))
                    qtn = tf.transformations.quaternion_from_euler(sum(r_w), sum(p_w), sum(yy_w))
                    if tagid0[0] > tagid[0]:
                        rospy.loginfo("请移步计算tag_" + str(tagid0[0]) + "与tag_" + str(tagid0[0] + 1) + "之间相对坐标")
                        f.write("tag_" + str(tagid0[0]) + "与tag_" + str(tagid0[0] - 1) + "之间相对坐标"+" "+
                                str(x_w[count-2])+" "+
                                str(y_w[count-2])+" "+
                                str(z_w[count-2])+" "+
                                str(r_w[count-2])+" "+
                                str(p_w[count-2])+" "+
                                str(yy_w[count-2])+" "+
                                "\n")
                        trajfile.write("tag_" + str(tagid0[0]) + " " +
                                       str(sum(x_w)) + " " + str(sum(y_w)) + " " + str(sum(z_w)) + " " +
                                       str(qtn[0]) + " " + str(qtn[1]) + " " + str(qtn[2]) + str(qtn[3]) + "\n")
                    else:
                        rospy.loginfo("请移步计算tag_" + str(tagid[0]) + "与tag_" + str(tagid[0] + 1) + "之间相对坐标")
                        f.write("tag_" + str(tagid[0]) + "与tag_" + str(tagid[0] - 1) + "之间相对坐标" + " " +
                                str(x_w[count - 2]) + " " +
                                str(y_w[count - 2]) + " " +
                                str(z_w[count - 2]) + " " +
                                str(r_w[count -2]) + " " +
                                str(p_w[count - 2]) + " " +
                                str(yy_w[count - 2]) + " " +
                                "\n")
                        trajfile.write("tag_" + str(tagid[0]) + " " +
                                       str(sum(x_w)) + " " + str(sum(y_w)) + " " + str(sum(z_w)) + " " +
                                       str(qtn[0]) + " " + str(qtn[1]) + " " + str(qtn[2]) + str(qtn[3]) + "\n")
                    # print(x_w)
                    # print(dx)
                    # rospy.loginfo(blurred)
                    # for i in range(100):
                    #     chazhi += ((blurred_x[i] - dx[i]) ** 2)
                    #     chazhi1 += ((blurred_y[i] - dy[i]) ** 2)
                    #     chazhi2 += ((blurred_z[i] - dz[i]) ** 2)
                    # var_x = np.var(dx)
                    # var_y = np.var(dy)
                    # var_z = np.var(dz)
                    # print(var_x)
                    # print(var_y)
                    # print(var_z)
                    # print(chazhi/100)
                    # print(chazhi1/100)
                    # print(chazhi2/100)
                    dx.clear()
                    dy.clear()
                    dz.clear()
                    # plt.plot(dx)
                    # plt.plot(blurred)
                    # plt.show()
                # else:
                #     print(i)
            else:
                print("-----------")
    except Exception as e:
        rospy.logwarn("++++++++++当前图像中并无tag信息，请调整摄像头！！！")
        rospy.logerr(e)

if __name__ == "__main__":
    # 2.初始化 ROS 节点
    rospy.init_node("sub_tf")
    sub = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, doPose)
    rospy.spin()
