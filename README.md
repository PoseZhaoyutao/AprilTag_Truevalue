# AprilTag_Truevalue
构建AprilTag真值系统，利用ROS通信，通过ZED相机对靶标采集图像流，解算首帧图像的精确位姿，以确定世界系原 点；通过铺设多靶标为场景提供伪真值，给出目标点位置信息，实时给出相机位置以测SLAM算法精度。
# tf_04test
tf_04中包含测量场地、首帧求解位姿、看任意靶标构网求解实时位姿，有兴趣的可以考虑加入自由网平差以降低测量误差带来的影响
# apriltag_ros_src&1launchfile
里面主要包含在ros通信时，对apriltag的部分修改以及启动文件中，直接copy进入自己的工程文件夹即可，（src与launch别放在一起哈！）
