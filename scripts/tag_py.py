import cv2
import numpy as np
from apriltag import apriltag
# @Author  : Zhao Yutao
# @Time    : 2024/7/1 12:29
# @Function: 测试redis服务端发送图像
# @mails: zhaoyutao22@mails.ucas.ac.cn
imagepath = '/usr/src/app/tag_image/0002.png'
image = cv2.imread(imagepath, cv2.IMREAD_GRAYSCALE)
detector = apriltag("tag36h11")

detections = detector.detect(image)
print(detections)
