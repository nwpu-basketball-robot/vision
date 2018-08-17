# 2018 图像

[TOC]

## 1 模型介绍

| 类别 | 说明                                                         |
| :--: | :----------------------------------------------------------- |
| 框架 | caffe，采用了SSD进行检测                                     |
| 硬件 | 神舟z8——gtx 1070                                             |
| 数据 | 输入数据为480*270的RGB图片                                   |
| 性能 | 充电时最高可达40fps，没有电源时只有8fps左右                  |
| 精度 | 总共比了36个回合，铲球132(理论值，回合有的没跑完，但因为赛前调试没有出过识别错误，所以可以类比)次，识别错2次，实测正确率高达**98.5%** |

## 2 环境配置（Ubuntu 16.04）

### 2.1 [ROS](http://wiki.ros.org/cn/kinetic/Installation/Ubuntu)

> 设置你的电脑可以从 packages.ros.org 接收软件.强烈建议使用国内或者新加波的镜像源，这样能够大大提高安装下载速度。

```shell
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```
> 添加 keys
```shell
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
```
> 安装
```shell
sudo apt-get update
```

```shell
sudo apt-get install ros-kinetic-desktop-full
```

> 初始化
```shell
sudo rosdep init
rosdep update
```
> 环境配置
```shell
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc #用zshrc的话，记得用.zshrc 以及 setup.zsh
```
> 构建工厂依赖

```shell
sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential
```

到此为止ROS已经安装完毕，此时**PCL**以及**OPENCV**都已经安装好了，所以即便不想用ROS，对你们来说，使用这种方法安装opencv也是最方便的。

### 2.2 [CUDA](https://developer.nvidia.com/cuda-downloads)

请按照官方说明下载并安装，并且保证CUDA版本和cudnn版本是对应的。如果资源下载速度慢，请用[蒲公英](https://npupt.com)进行搜索

安装完成后，请运行其测试程序验证安装有效性，并且在`.bashrc`或者`.zshrc`中添加下面的环境变量

```shell
export PATH=/usr/local/cuda/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH
```

### 2.3 [Caffe](https://github.com/weiliu89/caffe)

我使用的是weiliu89的caffe版本，和官方的有点区别，所以下载下来之后记得checkout。

首先的必要的环境配置，请参考[官方网站](http://caffe.berkeleyvision.org/install_apt.html)。

```shell
sudo apt install libprotobuf-dev libleveldb-dev libsnappy-dev libopencv-dev libhdf5-serial-dev protobuf-compiler
sudo apt install --no-install-recommends libboost-all-dev
sudo apt install libatlas-base-dev
sudo apt install the python-dev
```

除了上述的环境之外，还需要安装glog，gflags，lmdb，所以，分别执行以下命令。

```shell
sudo apt install libgoogle-glog-dev
sudo apt install libgflags-dev
sudo apt install liblmdb-dev
```

最后配置Caffe

```shell
git clone https://github.com/weiliu89/caffe.git
cd caffe
git checkout ssd
mkdir build && cd build
cmake ..
make all
make install
make runtest
```

runtest通过说明caffe配置没有问题。接下来给设置以下caffe的python环境变量，编辑你的`.zshrc`或者`.bashrc`，参考如下：

```shell
export PYTHONPATH=/home/shirlim/caffe/python:$PYTHONPATH
```

caffe有效性测试，CMakeLists.txt中默认的py2版本，请不要和anaconda或者是其他python环境冲突，在命令行中打开python：

```python
import caffe
caffe.__version__
```

如果有输出，代表配置完成，如果提示缺少某些库，请使用pip安装。

### 2.4 [libfreenect2](https://github.com/OpenKinect/libfreenect2)

对于kinect v2的配置。

```shell
删除git clone https://github.com/OpenKinect/libfreenect2.git
cd libfreenect2
sudo apt-get install build-essential cmake pkg-config
sudo apt-get install libusb-1.0-0-dev
sudo apt-get install libturbojpeg libjpeg-turbo8-dev
sudo apt-get install libglfw3-dev
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/freenect2 # 路径自定
make
make install
sudo cp ../platform/linux/udev/90-kinect2.rules /etc/udev/rules.d/
./bin/Protonect # test
```

## 3 训练方法(此部分应该自己探索)

我的建议是使用特定的尺寸，即把不同的图片缩放到固定的尺寸，主要为了保持球的边缘特征。

### 3.1 [标定图片](https://github.com/tzutalin/labelImg)

如果提示环境冲突，请新建一个用户，没必要重装系统。

标定图片注意不要标定互相重合面积过大(1/2)的。

### 3.2 生成数据

```shell
cd ~/data/VOCdevkit/VOC2007
python make_txt.py #生成处理列表，样本比例可调
cd $HOME
./caffe/data/.../create_list.sh #这两个脚本有些路径需要改的
./caffe/data/.../create_data.sh
```

### 3.3 训练网络

需要先学习一下神经网络的基础知识，否则乱改一段时间代码就崩掉了。

除了这个[细节参考](https://blog.csdn.net/u013738531/article/details/58637760)之外，还要注意一个叫做`min_dim`的参数，这个参数指的是输入图片的高度。测试阶段需要用到。

```shell
cd $HOME/caffe
python examples/ssd/ssd_pascal.py #开始训练
```

### 3.4 测试模型

可以先使用`ssd_pascal_webcam.py`测试，发现没有过拟合或者其他问题就可以进行下一步的工作。

作者提供了一个cpp文件，里面留了cpp接口，我把它稍微改了以下，调用方式保持不变。见`ssd_detect.cpp`

```shell
cd $HOME/caffe
python examples/ssd/ssd_pascal_webcam.py #开始测试
rosrun vision judge # 篮球ROS通信中视觉单独测试模块
```

## 4 图像的一点总结

### 4.1 关于标定数据

数据最起码的要求是类别要对，其次是框的精度。

针对于我们篮球组的特定问题，在标定的时候尽量不要把过远处的球放进来，尽量不要把模糊的球放进来，尽量保持数据分布的一致性：即最好用一个人的手机去拍照，不同手机的质量相差很大，图片在处理之后差距很大。

### 4.2 关于任务分工

状态机负责电子机械以及图像三个方面的联合调试，任何方面出了问题，都会影响到状态机的调试过程（没钱不能保证精确性，出问题是肯定的），所以图像作为软件组的一员，不能仅仅只做完识别，最好负责一下距离解算之类的工作。2019软件组有两个人，希望互相合作理解。

### 4.3 关于机器依赖程度

深度学习是一个傻瓜式的拟合过程，不需要懂多少图像处理的东西就可以上手，所以导致很多人觉得学习纯CV没有什么用处。事实上，只掌握这种算法并不能算上是做图像的人员。这也导致你们前期学习的OpenCV和后面的深度学习跃度有点大。下一届是否要用深度学习算法还是要看自己怎么想，硬件永远不是问题，也不是说除了篮球组别的地方就没有应用深度学习的地方。