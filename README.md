# vision
Vision for nwpu-basketball-robot

1）工程简介
basketball_msgs为ROS通讯包，包含相关消息和服务的定义。
driver_common-indigo-devel和hokuyo_node-indigo-devel为激光雷达驱动，包含激光雷达相关接口，
在官方提供的驱动源码中作了修改，在hokuyo_node.h中声明了一个dists数组用于雷达数据的直接读取。
vision为视觉部分的代码，是工程的主要部分。 
标定柱的识别使用纯opencv实现
四种球的识别使用深度学习（yolov3）
角度解算使用纯数学方法 
测距使用激光雷达

2）依赖项
ROS-kinetic yolov3  opencv
yolov3的源码：
https://github.com/pjreddie/darknet

3）编译方法
编译testusbcap，将生成的lib文件（UsbCapture）添加到/usr/local/lib中
编译yolov3，将生成的lib文件（darknet）添加到/usr/local/lib中
然后在testvision目录下
sudo ldconfig 
catkin_make
即可

3）总结
视觉模块代码的帧率十分关键，机器人不能准确转向球可能就是因为图像节点和状态机之间的数据不同步。
电脑插电跑机器人可以一次性转到位，不插电可能需要二次对准。

4）一些未实现的想法
达到极致帧率的方法可能是，用激光雷达数据做一个区域建议，把每个框出来的图像走纯CV做分类。
目前已经实现的是通过雷达数据标定出球的左右边界，但是上下边界标定不出
可以证明的是放在水平面上的所有球在摄像头里成像的球心是共一条水平线的，根据这个可以标定出球的上下边界
