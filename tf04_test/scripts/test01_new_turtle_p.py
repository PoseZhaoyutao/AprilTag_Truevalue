#! /usr/bin/env python
# @Author  : Zhao Yutao
# @Time    : 2024/7/1 12:29
# @Function: 测试redis服务端发送图像
# @mails: zhaoyutao22@mails.ucas.ac.cn
import rospy
from turtlesim.srv import *

"""
    需求：生成一只小乌龟
    话题：/spawn
    消息：turtlesim/Spawn

    1.导包
    2。初始化
    3.创建服务的客户端对象
    4.组织数据并发送请求
    5.处理响应结果
"""

if __name__ == '__main__':
    rospy.init_node('service_call_p')
    client = rospy.ServiceProxy("/spawn", Spawn)
    request = SpawnRequest()
    request.x = 4.5
    request.y = 2.0
    request.theta = -3
    request.name = "turtle2"

    client.wait_for_service()
    try:
        response = client.call(request)
    except Exception as e:
        print(e)
