[TOC]

------

# SnakeSys

<p align="center">
	<a href="https://github.com/LiuXPs/SnakeSys"><img src="https://img.shields.io/badge/GitHub-SnakeSys-yellow?style=flat&logo=GitHub&logoColor=yellow"></a>
    <a href="https://github.com/LiuXPs/SnakeSys/blob/main/LICENSE"><img src="https://img.shields.io/badge/License-GPLv3-blue?style=flat&logo=gnu&logoColor=blue"></a>
</p>



`SnakeSys` is an open-source system for snake robots.



## v1.0, December 22th, 2024

Authors: Liu Xupeng, [Zang Yong](https://me.ustb.edu.cn/shiziduiwu/jiaoshixinxi/2022-03-24/436.html), and [Gao Zhiying](https://me.ustb.edu.cn/shiziduiwu/jiaoshixinxi/2022-03-24/601.html).



> [!IMPORTANT]
>
> `SnakeSys` includes five modules: CPG-based multimodal motion control, prototypes of snake robots, Gazebo-based simulation, information perception, and GUI. The source code of `SnakeSys` is in the folders `SnakeSys/ros2/src/snake` and `SnakeSys/ros1/src/snake`.

<div style="display: flex; justify-content: space-around;">
    <img src="https://raw.githubusercontent.com/LiuXPs/PicBed/main/typora_img/Snake3_Sim_CL.png" alt="Image_1" style="width:19%; height:auto;">
    <img src="https://raw.githubusercontent.com/LiuXPs/PicBed/main/typora_img/Snake3_Sim_TWL.png" alt="Image_2" style="width:19%; height:auto;">
    <img src="https://raw.githubusercontent.com/LiuXPs/PicBed/main/typora_img/Snake3_Sim_SWL.png" alt="Image_3" style="width:19%; height:auto;">
    <img src="https://raw.githubusercontent.com/LiuXPs/PicBed/main/typora_img/Snake3_Sim_ARL.png" alt="Image_4" style="width:19%; height:auto;">
    <img src="https://raw.githubusercontent.com/LiuXPs/PicBed/main/typora_img/Snake3_Sim_SRL.png" alt="Image_5" style="width:19%; height:auto;">
</div>
If you use `SnakeSys` in an academic work, please cite:

```latex
@article{liu2024bio,
  title={Bio-Inspired Multimodal Motion Gait Control of Snake Robots with Environmental Adaptability Based on ROS},
  author={Liu, Xupeng and Zang, Yong and Gao, Zhiying},
  journal={Electronics},
  volume={13},
  number={17},
  pages={3437},
  year={2024},
  publisher={MDPI}
}
```

```latex
@article{liu2024locomotion,
  title={Locomotion gait control of snake robots based on a novel unified CPG network model composed of Hopf oscillators},
  author={Liu, Xupeng and Zang, Yong and Gao, Zhiying and Liao, Maolin},
  journal={Robotics and Autonomous Systems},
  volume={179},
  pages={104746},
  year={2024},
  publisher={Elsevier}
}
```

```latex
@article{liu2021tribological,
  title={Tribological Mechanism and Propulsion Conditions for Creeping Locomotion of the Snake-like Robot},
  author={Liu, Xupeng and Gao, Zhiying and Zang, Yong and Zhang, Liyuan},
  journal={Journal of Mechanical Engineering},
  volume={57},
  number={21},
  pages={189--201},
  year={2021}
}
% or
@article{刘旭鹏2021蛇形机器人蜿蜒运动的摩擦机理及推进条件,
  title={蛇形机器人蜿蜒运动的摩擦机理及推进条件},
  author={刘旭鹏 and 郜志英 and 臧勇 and 张立元},
  journal={机械工程学报},
  volume={57},
  number={21},
  pages={189--201},
  year={2021}
}
```

------

## Hardware prerequisites

### Actuator

<p align="left">
	<a href="https://www.feetech.cn/"><img src="https://img.shields.io/badge/FEETECH SM8524BL Servo-Org-orange?style=flat&logo=flipkart&logoColor=orange"></a>
    <a href="https://gitee.com/ftservo"><img src="https://img.shields.io/badge/FEETECH Servo SDK-Gitee-orange?style=flat&logo=gitee&logoColor=orange"></a>
</p>



<div style="display: flex; justify-content: space-around;">
    <img src="https://raw.githubusercontent.com/LiuXPs/PicBed/main/typora_img/feetech_servo_01.jpg" alt="Image_1" style="width:20%; height:auto;">
</div>


> [!IMPORTANT]
>
> We have built the servo driver for ROS2 in folder `SnakeSys/ros2/src/snake/snake_servo`.

### Controller

<p align="left">
	<a href="https://www.lattepanda.com/"><img src="https://img.shields.io/badge/LattePanda 3 Delta-Org-yellow?style=flat&logo=foodpanda&logoColor=yellow"></a>
</p>


<div style="display: flex; justify-content: space-around;">
    <img src="https://raw.githubusercontent.com/LiuXPs/PicBed/main/typora_img/LattePanda_3_Delta.png" alt="Image_1" style="width:30%; height:auto;">
</div>



### Visual sensor

<p align="left">
	<a href="https://www.intelrealsense.com/depth-camera-d455/"><img src="https://img.shields.io/badge/Intel RealSense D455-Org-yellow?style=flat&logo=intel&logoColor=yellow"></a>
    <a href="https://www.stereolabs.com/"><img src="https://img.shields.io/badge/StereoLabs ZED 2i-Org-blue?style=flat&logo=sanity&logoColor=blue"></a>
    <a href=""><img src="https://img.shields.io/badge/USB Stereo-Org-green?style=flat&logo=ubiquiti&logoColor=green"></a>
</p>


<div style="display: flex; justify-content: space-around;">
    <img src="https://raw.githubusercontent.com/LiuXPs/PicBed/main/typora_img/depth-camera-d455-intel-realsense.jpg" alt="Image_1" style="width:30%; height:auto;">
    <img src="https://raw.githubusercontent.com/LiuXPs/PicBed/main/typora_img/ZED%202i.jpg" alt="Image_2" style="width:30%; height:auto;">
    <img src="https://raw.githubusercontent.com/LiuXPs/PicBed/main/typora_img/usb_stereo_4.jpg" alt="Image_3" style="width:30%; height:auto;">
</div>



> [!IMPORTANT]
>
> In folders `SnakeSys/ros2/src/sensor_camera/realsense_cam`, `SnakeSys/ros2/src/sensor_camera/zed_cam`, and `SnakeSys/ros2/src/sensor_camera/usb_cam`.

### Inertial sensor

<p align="left">
	<a href="https://www.wheeltec.net/"><img src="https://img.shields.io/badge/WheelTec N100-Org-blue?style=flat&logo=intel&logoColor=blue"></a>
</p>


<div style="display: flex; justify-content: space-around;">
    <img src="https://raw.githubusercontent.com/LiuXPs/PicBed/main/typora_img/IMU%20N100%202.jpg" alt="Image_1" style="width:30%; height:auto;">
</div>


> [!IMPORTANT]
>
> In folder `SnakeSys/ros2/src/sensor_imu/fdilink_ahrs`.

### Joystick

<p align="left">
	<a href="https://wiki.ros.org/joy"><img src="https://img.shields.io/badge/Logitech F710-Docs-yellow?style=flat&logo=ROS&logoColor=yellow"></a>
</p>



<div style="display: flex; justify-content: space-around;">
    <img src="https://raw.githubusercontent.com/LiuXPs/PicBed/main/typora_img/logitech_1.svg" alt="Image_1" style="width:30%; height:auto;">
    <img src="https://raw.githubusercontent.com/LiuXPs/PicBed/main/typora_img/logitech_2.svg" alt="Image_2" style="width:30%; height:auto;">
    <img src="https://raw.githubusercontent.com/LiuXPs/PicBed/main/typora_img/logitech_3.svg" alt="Image_3" style="width:30%; height:auto;">
</div>


------

## Software prerequisites

### Ubuntu 20.04 + ROS1 Noetic + ROS2 Foxy

We used the `Ubuntu 20.04` desktop for 64-bit, `ROS1 Noetic` and `ROS2 Foxy`, and used the `zsh` terminal.



#### Install Ubuntu 20.04

<p align="left">
    <a href="https://mirrors.tuna.tsinghua.edu.cn/ubuntu-releases/20.04/"><img src="https://img.shields.io/badge/Ubuntu 20.04-Downloads-orange?style=flat&logo=Ubuntu&logoColor=orange"></a>
</p>


#### Install `zsh` terminal

<p align="left">
    <a href="https://zsh.sourceforge.io/"><img src="https://img.shields.io/badge/zsh-Org-green?style=flat&logo=zsh&logoColor=green"></a>
</p>



```shell
# Check the current shell in the system.
$ echo $SHELL

# Check the shells in the system.
$ cat /etc/shells

# Install zsh shell.
$ sudo apt-get install zsh -y

# Set zsh as the default shell.
$ chsh -s /bin/zsh

# Install oh-my-zsh
$ sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)"

# Restart the Ubuntu.
$ sudo reboot

# Configure the environment.
$ source ~/.zshrc
```

#### Install and configure`ROS1 Noetic`

<p align="left">
    <a href="https://wiki.ros.org/noetic"><img src="https://img.shields.io/badge/ROS1-Noetic-yellow?style=flat&logo=ROS&logoColor=yellow"></a>
</p>



##### Install `ROS1 Noetic`

```shell
# Setup Sources.
$ sudo sh -c '. /etc/lsb-release && echo "deb http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'

# Set up your keys.
$ curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Installation.
$ sudo apt update
$ sudo apt install ros-noetic-desktop-full

# Environment setup.
$ echo "source /opt/ros/noetic/setup.zsh" >> ~/.zshrc
$ source ~/.zshrc
```

##### Configure the environment

```shell
$ vim ~/.zshrc
```

```shell
# Write the following content in .zshrc.
# ******************** ros1 ********************
source /opt/ros/noetic/setup.zsh
##################################################
```

```shell
$ source ~/.zshrc
```

##### Install dependency packages

```shell
$ sudo apt install python3-rosdep
$ sudo rosdep init
$ rosdep update

$ rosdep install --from-paths src --ignore-src -r -y
```

##### Install Python3 packages

```shell
$ sudo apt-get install build-essential
$ pip3 install rosdep rosinstall rosinstall-generator wstool vcstools vcstool
$ pip3 install rospkg catkin_pkg pyyaml empy numpy defusedxml
```

##### Install necessary packages

```shell
$ sudo apt-get install ros-noetic-ddynamic-reconfigure
$ sudo apt-get install ros-noetic-position-controllers
$ sudo apt-get install ros-noetic-effort-controllers
$ sudo apt-get install ros-noetic-joint-state-controller
$ sudo apt-get install ros-noetic-control*
$ sudo apt-get install ros-noetic-serial*
$ sudo apt-get install ros-noetic-usb-cam*
$ sudo apt-get install ros-noetic-cv-bridge*
$ sudo apt-get install ros-noetic-cv-camera*
$ sudo apt-get install ros-noetic-dynamixel*
$ sudo apt-get install ros-noetic-xacro
$ sudo apt-get install ros-noetic-rviz*
$ sudo apt-get install ros-noetic-rqt*
$ sudo apt-get install ros-noetic-urdf*
$ sudo apt-get install ros-noetic-joy
$ sudo apt-get install ros-noetic-tf*
```

#### Install and configure `ROS2 Foxy`

<p align="left">
    <a href="https://docs.ros.org/en/foxy/"><img src="https://img.shields.io/badge/ROS2-Foxy-blue?style=flat&logo=ROS&logoColor=blue"></a>
</p>



##### Install `ROS2 Foxy`

```shell
# Set locale.
$ locale  # check for UTF-8
$ sudo apt update && sudo apt install locales
$ sudo locale-gen en_US en_US.UTF-8
$ sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
$ export LANG=en_US.UTF-8
# Verify settings.
$ locale

# Setup Sources.
# First ensure that the Ubuntu Universe repository is enabled.
$ sudo apt install software-properties-common
$ sudo add-apt-repository universe
# Now add the ROS2 GPG key with apt.
$ sudo apt update && sudo apt install curl
$ sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
# Then add the repository to your sources list.
$ sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://mirror.tuna.tsinghua.edu.cn/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list'

# Install ROS2 Packages.
$ sudo apt update
$ sudo apt upgrade
$ sudo apt install ros-humble-desktop-full

# Environment Setup.
$ echo "source /opt/ros/humble/setup.zsh" >> ~/.zshrc
$ source ~/.zshrc

# Check environment variables.
$ printenv | grep -i ROS
```

##### Configure the environment

```shell
$ vim ~/.zshrc
```

```shell
# Write the following content in .zshrc.
# ******************** ros2 ********************
source /opt/ros/humble/setup.zsh
export ROS_DOMAIN_ID=<your_domain_id>
##################################################
```

```shell
$ source ~/.zshrc
```

##### Install dependency packages

```shell
$ sudo apt-get install python3-rosdep

$ pip3 install -U rosdep
$ sudo rosdep init
$ rosdep update

$ rosdep install -i --from-path src --rosdistro foxy -y
```

##### Install Python3 packages

```shell
$ sudo apt-get install python3-argcomplete
$ sudo apt-get install python3-colcon-common-extensions
$ pip3 install -U colcon-common-extensions

$ sudo apt-get install build-essential
$ sudo apt-get install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool python3-vcstool
$ sudo apt-get install python3-rospkg python3-empy python3-numpy
$ sudo apt-get install python3-setuptools python3-pkg-resources python3-lark
$ pip3 install -U catkin_pkg pyyaml lark-parser pyyaml

$ sudo apt-get install build-essential
$ pip3 install -U rosdep rosinstall rosinstall-generator wstool vcstool
$ pip3 install -U rospkg catkin_pkg pyyaml empy numpy
$ pip3 install -U setuptools lark lark-parser
```

##### Install necessary packages

```shell
$ sudo apt-get install ros-foxy-position-controllers
$ sudo apt-get install ros-foxy-effort-controllers
$ sudo apt-get install ros-foxy-joint-state-controller
$ sudo apt-get install ros-foxy-control*
$ sudo apt-get install ros-foxy-serial*
$ sudo apt-get install ros-foxy-usb-cam*
$ sudo apt-get install ros-foxy-cv-bridge*
$ sudo apt-get install ros-foxy-dynamixel*
$ sudo apt-get install ros-foxy-xacro
$ sudo apt-get install ros-foxy-rviz*
$ sudo apt-get install ros-foxy-rqt*
$ sudo apt-get install ros-foxy-urdf*
$ sudo apt-get install ros-foxy-joy*
$ sudo apt-get install ros-foxy-tf*
$ sudo apt-get install ros-foxy-gazebo*
$ sudo apt-get install ros-foxy-nmea-msgs ros-foxy-geographic-msgs ros-foxy-robot-localization ros-foxy-robot-localization ros-foxy-image-transport-plugins ros-foxy-realsense2-camera
```

### C++17 Compiler

```shell
# Install g++-9 and gcc-9.
$ sudo apt-get install g++-9 gcc-9

# Check the current version.
$ gcc --version
$ g++ --version
```

### Python 3.8 and Python 3.10

<p align="left">
	<a href="https://www.anaconda.com/download/"><img src="https://img.shields.io/badge/Anaconda3-Org-green?style=flat&logo=anaconda"></a>
</p>



We used `Anaconda3 2024.02` to create two environments: `Python 3.8` and `Python 3.10`.

```shell
# Create environments.
$ conda create -n python38 python=3.8 # For ROS2.
$ conda create -n python310 python=3.10 # For ROS1.

# Delete environments.
$ conda remove --name python38 --all
$ conda remove --name python310 --all

# Activate the environment.
$ conda activate python38
$ conda activate python310

# Close the environment.
$ conda deactivate
```

### CMake 3.29.3

<p align="left">
	<a href="https://cmake.org/download/"><img src="https://img.shields.io/badge/CMake-Downloads-red?style=flat&logo=CMake&logoColor=red"></a>
    <a href="https://cmake.org/documentation/"><img src="https://img.shields.io/badge/CMake-Docs-red?style=flat&logo=CMake&logoColor=red"></a>
</p>



```shell
# Download CMake.
$ wget https://github.com/Kitware/CMake/releases/download/v3.29.3/cmake-3.29.3.tar.gz
$ tar -zxvf cmake-3.29.3.tar.gz
$ sudo cp -r cmake-3.29.3 /opt
$ cd /opt/cmake-3.29.3

# Configure.
$ sudo ./configure -- -DCMAKE_USE_OPENSSL=OFF

# Compile.
$ sudo make -j$(nproc)

# Install.
$ sudo make install
```

### Qt 6.7.0 and QCustomPlot 2.1.1

<p align="left">
	<a href="https://download.qt.io/archive/qt/"><img src="https://img.shields.io/badge/Qt-Downloads-green?style=flat&logo=Qt&logoColor=green"></a>
    <a href="https://www.qcustomplot.com/index.php/download"><img src="https://img.shields.io/badge/qcustomplot-Downloads-green?style=flat&logo=Qt&logoColor=green"></a>
</p>



> [!IMPORTANT]
>
> We have added the `qcustomplot` to `SnakeSys/ros2/src/snake/snake_gui/include/snake_gui/qcustomplot.h` and `SnakeSys/ros2/src/snake/snake_gui/src/qcustomplot.cpp` of `SnakeSys`.


#### Install dependencies

```shell
$ sudo apt-get install libgl1-mesa-dev fcitx-frontend-qt5 libmd4c-dev
$ sudo apt-get install libfontconfig1-dev libfreetype6-dev libx11-dev libxext-dev libxfixes-dev libxi-dev libxrender-dev libxcb1-dev libx11-xcb-dev libxcb-glx0-dev libxkbcommon-x11-dev
$ sudo apt-get install libxcb-keysyms1-dev libxcb-image0-dev libxcb-shm0-dev libxcb-icccm4-dev libxcb-sync0-dev libxcb-xfixes0-dev libxcb-shape0-dev libxcb-randr0-dev libxcb-render-util0-dev libxcb-cursor0
```

#### Install Qt 6.7.0

#### Configure the environment

```shell
$ vim ~/.zshrc
```

```shell
# Write the following content in .zshrc.
# ******************** qt ********************
# export QT_DEBUG_PLUGINS=1
export QT6_HOME=$QT6_HOME:/opt/Qt/6.7.0
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$QT6_HOME/gcc_64/lib
##############################################
```

```shell
$ source ~/.zshrc
```

### Boost 1.85.0

<p align="left">
	<a href="https://www.boost.org/users/download/"><img src="https://img.shields.io/badge/Boost-Downloads-blue?style=flat&logo=Boost&logoColor=blue"></a>
    <a href="https://www.boost.org/doc/libs/1_85_0"><img src="https://img.shields.io/badge/Boost-Docs-blue?style=flat&logo=Boost&logoColor=blue"></a>
</p>



#### Install dependencies

```shell
$ sudo apt-get install build-essential g++ autotools-dev libicu-dev libbz2-dev
```

#### Compile

```shell
# Download Boost.
$ wget https://archives.boost.io/release/1.85.0/source/boost_1_85_0.tar.gz
$ tar -zxvf boost_1_85_0.tar.gz
$ sudo cp -r boost_1_85_0 /opt
$ cd /opt/boost_1_85_0

# Compile.
$ sudo ./bootstrap.sh --with-python=/opt/anaconda3/bin/python3.11 --with-toolset=gcc
$ sudo ./b2 --with-python include="/opt/anaconda3/include/python3.11" --with-atomic --with-log --with-test --with-thread --with-date_time --with-chrono

# Install.
$ sudo ./b2 install
```

#### Configure

```shell
$ cd /usr/local/lib
$ sudo ln -s libboost_python311.so libboost_python3.so
$ sudo ln -s libboost_python311.a libboost_python3.a
```

### GSL 2.7.1

<p align="left">
	<a href="https://mirrors.tuna.tsinghua.edu.cn/gnu/gsl/"><img src="https://img.shields.io/badge/GSL-Downloads-yellow?style=flat&logo=gnu&logoColor=yellow"></a>
    <a href="https://www.gnu.org/software/gsl/doc/html/index.html"><img src="https://img.shields.io/badge/GSL-Docs-yellow?style=flat&logo=gnu&logoColor=yellow"></a>
</p>



```shell
# Download Boost.
$ wget https://mirrors.tuna.tsinghua.edu.cn/gnu/gsl/gsl-2.7.1.tar.gz
$ tar -zxvf gsl-2.7.1.tar.gz
$ sudo cp -r gsl-2.7.1 /opt
$ cd /opt/gsl-2.7.1

# Compile.
$ sudo ./configure
$ sudo make -j$(nproc)

# Install.
$ sudo make install
```

### Eigen 3.4.0

<p align="left">
	<a href="http://eigen.tuxfamily.org"><img src="https://img.shields.io/badge/Eigen3-Downloads-green?style=flat&logo=etsy&logoColor=green"></a>
</p>



```shell
# Download Eigen3.
$ wget https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
$ tar -zxvf eigen-3.4.0.tar.gz
$ sudo cp -r eigen-3.4.0 /opt
$ cd /opt/eigen-3.4.0

# Compile.
$ sudo mkdir build
$ cd build
$ sudo ccmake ..
$ sudo make -j$(nproc)

# Install.
$ sudo make install
```

### fmt 10.2.1

<p align="left">
	<a href="https://github.com/fmtlib/fmt.git"><img src="https://img.shields.io/badge/GitHub-Downloads-blue?style=flat&logo=GitHub&logoColor=blue"></a>
</p>



```shell
# Download fmt.
$ wget https://github.com/fmtlib/fmt/releases/download/10.2.1/fmt-10.2.1.zip
$ unzip fmt-10.2.1.zip
$ sudo cp -r fmt-10.2.1 /opt
$ cd /opt/fmt-10.2.1

# Compile.
$ sudo mkdir build
$ cd build
$ sudo ccmake ..
$ sudo make -j$(nproc)

# Install.
$ sudo make install
```

### Sophus 1.22.10

<p align="left">
    <a href="https://github.com/strasdat/Sophus"><img src="https://img.shields.io/badge/GitHub-Downloads-orange?style=flat&logo=GitHub&logoColor=orange"></a>
</p>


```shell
# Download Sophus.
$ wget https://github.com/strasdat/Sophus/archive/refs/tags/1.22.10.tar.gz
$ tar -zxvf 1.22.10.tar.gz
$ sudo cp -r Sophus-1.22.10 /opt
$ cd /opt/Sophus-1.22.10

# Compile.
$ sudo mkdir build
$ cd build
$ sudo ccmake ..
$ sudo make -j$(nproc)

# Install.
$ sudo make install
```

### VTK 9.3.0

<p align="left">
    <a href="https://vtk.org/download/"><img src="https://img.shields.io/badge/VTK-Downloads-yellow?style=flat&logo=vala&logoColor=yellow"></a>
    <a href="https://vtk.org/documentation/"><img src="https://img.shields.io/badge/VTK-Docs-yellow?style=flat&logo=vala&logoColor=yellow"></a>
</p>


```shell
# Download VTK.
$ wget https://www.vtk.org/files/release/9.3/VTK-9.3.0.tar.gz
$ wget https://www.vtk.org/files/release/9.3/VTKData-9.3.0.tar.gz
$ wget https://www.vtk.org/files/release/9.3/VTKDataFiles-9.3.0.tar.gz
$ wget https://www.vtk.org/files/release/9.3/VTKLargeData-9.3.0.tar.gz
$ wget https://www.vtk.org/files/release/9.3/VTKLargeDataFiles-9.3.0.tar.gz

$ tar -zxvf VTK-9.3.0.tar.gz
$ tar -zxvf VTKData-9.3.0.tar.gz
$ tar -zxvf VTKDataFiles-9.3.0.tar.gz
$ tar -zxvf VTKLargeData-9.3.0.tar.gz
$ tar -zxvf VTKLargeDataFiles-9.3.0.tar.gz

$ sudo cp -r VTK-9.3.0 /opt
$ cd /opt/VTK-9.3.0

$ sudo mkdir build
$ cd build
$ sudo ccmake ..
```

```shell
# Configure.
BUILD_SHARED_LIBS=ON
CMAKE_BUILD_TYPE=Release
VTK_BUILD_DOCUMENTATION=ON
VTK_BUILD_EXAMPLES=ON
VTK_BUILD_TESTING=ON
VTK_FORBID_DOWNLOADS=ON
VTK_USE_LARGE_DATA=ON

# Qt options.
VTK_GROUP_ENABLE_Qt=YES
VTK_QT_VERSION=6
VTK_MODULE_ENABLE_VTK_GuiSupportQt=YES
VTK_MODULE_ENABLE_VTK_GuiSupportQtQuick=YES
VTK_MODULE_ENABLE_VTK_GuiSupportQtSQL=YES
VTK_MODULE_ENABLE_VTK_RenderingQt=YES
VTK_MODULE_ENABLE_VTK_ViewsQt=YES

# gcc.
CMAKE_CXX_COMPILER_AR=/usr/bin/gcc-ar-9
CMAKE_CXX_COMPILER_RANLIB=/usr/bin/gcc-ranlib-9
CMAKE_C_COMPILER_AR=/usr/bin/gcc-ar-9
CMAKE_C_COMPILER_RANLIB=/usr/bin/gcc-ranlib-9

# CUDA.
CMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc
VTK_USE_CUDA=ON

# Python3.
Python3_EXECUTABLE=/opt/anaconda3/bin/python3
```

```shell
# Compile.
$ sudo cmake -Wno-dev ..
$ sudo make -j$(nproc)

# Install.
$ sudo make install
```

### OpenCV 4.9.0

<p align="left">
    <a href="https://opencv.org/"><img src="https://img.shields.io/badge/OpenCV-Org-blue?style=flat&logo=opencv&logoColor=blue"></a>
    <a href="https://docs.opencv.org/"><img src="https://img.shields.io/badge/OpenCV-Docs-blue?style=flat&logo=gnu&logoColor=blue"></a>
    <a href="https://github.com/opencv/opencv"><img src="https://img.shields.io/badge/GitHub-opencv-blue?style=flat&logo=GitHub&logoColor=blue"></a>
    <a href="https://github.com/opencv/opencv_contrib"><img src="https://img.shields.io/badge/GitHub-opencv_contrib-blue?style=flat&logo=GitHub&logoColor=blue"></a>
</p>


```shell
# Source code directory structure:
/opt/opencv
├── build
├── opencv-4.9.0
└── opencv_contrib-4.9.0

$ cd build
$ sudo cmake-gui ..
```

```shell
# Configure.
BUILD_EXAMPLES=ON
BUILD_opencv_world=ON
CMAKE_BUILD_TYPE=Release
OPENCV_ENABLE_NONFREE=ON
OPENCV_EXTRA_MODULES_PATH=/opt/opencv/opencv_contrib-4.9.0/modules
WITH_V4L=ON
WITH_OPENGL=ON
VTK_DIR=/usr/local/lib/cmake/vtk-9.3

WITH_CUDA=ON
# WITH_QT=ON
```

```shell
# Compile.
$ sudo ccmake ../opencv-4.9.0
$ sudo make -j$(nproc)

# Install.
$ sudo make install
```

### PCL 1.14.1

<p align="left">
    <a href="https://pointclouds.org/"><img src="https://img.shields.io/badge/PCL-Org-green?style=flat&logo=piapro&logoColor=green"></a>
    <a href="https://github.com/PointCloudLibrary/pcl"><img src="https://img.shields.io/badge/GitHub-Downloads-green?style=flat&logo=GitHub&logoColor=green"></a>
</p>


```shell
# Download PCL.
$ wget https://github.com/PointCloudLibrary/pcl/archive/refs/tags/pcl-1.14.1.tar.gz
$ tar -zxvf pcl-1.14.1.tar.gz
$ sudo cp -r pcl-1.14.1 /opt
$ cd /opt/pcl-1.14.1

$ sudo mkdir build
$ cd build
$ sudo ccmake ..
```

```shell
# Configure.
BUILD_examples=ON
BUILD_CUDA=ON
BUILD_GPU=ON
CMAKE_BUILD_TYPE=Release
WITH_QT=QT6
WITH_CUDA=ON
CMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc
```

```shell
# Compile.
$ sudo make -j$(nproc)

# Install.
$ sudo make instal
```

### ORB-SLAM3

<p align="left">
	<a href="https://github.com/UZ-SLAMLab/ORB_SLAM3/tree/c%2B%2B14_comp"><img src="https://img.shields.io/badge/GitHub-Downloads-yellow?style=flat&logo=GitHub&logoColor=yellow"></a>
</p>



> [!IMPORTANT]
>
> In folder `SnakeSys/ORB_SLAM3`.

------

## Building `SnakeSys`

`SnakeSys` includes ROS1 and ROS2 packages, which need to be compiled separately.

```shell
# Download the source code on your computer and the controller of the snake robot.
$ git clone https://github.com/LiuXPs/SnakeSys.git
```

### Build for ROS2 packages

ROS2 packages are divided into two parts: computer and controller. Due to the software and hardware requirements, not all packages can be compiled on the controller.

#### On computer

```shell
$ cd SnakeSys/ros2

# Set up the environment.
$ source /opt/ros/foxy/setup.zsh
$ conda activate python38

# Compile all packages for the computer.
$ colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --parallel-workers $(nproc)
```

#### On controller

```shell
$ cd SnakeSys/ros2

# Set up the environment.
$ source /opt/ros/foxy/setup.zsh
$ conda activate python38

# Compile corresponding packages based on the nodes deployed on the controller.
$ colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --parallel-workers $(nproc) --packages-select usb_cam fdilink_ahrs snake_cpg snake_frame snake_heart snake_joy snake_msgs snake_params snake_sensor snake_servo
```

### Build for ROS1 packages

ROS1 packages are only used to simulate the snake robot, and you needn't compile it if you don't simulate it.

#### On computer

```shell
$ cd SnakeSys/ros1

# Set up the environment.
$ source /opt/ros/noetic/setup.zsh
$ conda activate python310

# Compile the package for the computer.
$ catkin_make -j$(nproc) -DCMAKE_BUILD_TYPE=Release
```

------

## Running `SnakeSys`

Run `SnakeSys` for the prototype or simulation.

### For the prototype

Start `terminal 1` on the controller of the snake robot with ROS2.

```shell
# Start snake_heart, snake_servo, and snake_frame.
$ cd SnakeSys/ros2
$ source /opt/ros/foxy/setup.zsh
$ . install/setup.zsh

$ ls /dev/ttyUSB*
$ sudo chmod 777 /dev/ttyUSB*

# Need to confirm the serial port number [/dev/ttyUSBx] of your device.
$ ros2 launch snake_servo snake_servo.launch.py use_sim_time:=false serial_port:=/dev/ttyUSB0 heart_rate:=50
```

Start `terminal 2` on the computer or controller with ROS2.

```shell
# Start snake_cpg_hopf.
$ cd SnakeSys/ros2
$ source /opt/ros/foxy/setup.zsh
$ . install/setup.zsh
$ ros2 launch snake_cpg snake_cpg_hopf.launch.py use_sim_time:=false
```

Start `terminal 3` on the computer with ROS2.

```shell
# Start snake_gui.
$ cd SnakeSys/ros2
$ source /opt/ros/foxy/setup.zsh
$ . install/setup.zsh
$ ros2 launch snake_gui snake_gui.launch.py use_sim_time:=false
```

### For the simulation

Start `terminal 1` on the computer with ROS1.

```shell
# Start gazebo_sim.
$ cd SnakeSys/ros1
$ source /opt/ros/noetic/setup.zsh
$ conda activate python310
$ source devel/setup.zsh
$ roslaunch snake_sim snake_gazebo.launch use_sim_time:=true
```

Start `terminal 2` on the computer with ROS2.

```shell
# Start snake_sim.
$ cd SnakeSys/ros2
$ source /opt/ros/foxy/setup.zsh
$ . install/setup.zsh
$ ros2 launch snake_sim snake_sim.launch.py use_sim_time:=true heart_rate:=50
```

Start `terminal 3` on the computer with ROS1 and ROS2.

```shell
# Start ros1_bridge.
$ cd SnakeSys/ros2
$ source /opt/ros/noetic/setup.zsh
$ source /opt/ros/foxy/setup.zsh
$ ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
```

Start `terminal 4` on the computer with ROS2.

```shell
# Start snake_cpg_hopf.
$ cd SnakeSys/ros2
$ source /opt/ros/foxy/setup.zsh
$ . install/setup.zsh
$ ros2 launch snake_cpg snake_cpg_hopf.launch.py use_sim_time:=true
```

Start `terminal 5` on the computer with ROS2.

```shell
# Start snake_gui.
$ cd SnakeSys/ros2
$ source /opt/ros/foxy/setup.zsh
$ . install/setup.zsh
$ ros2 launch snake_gui snake_gui.launch.py use_sim_time:=true
```

------

## License

1. `SnakeSys` is released under the [GPLv3 license](https://www.gnu.org/licenses/gpl-3.0.html).
2. Source codes in files `SnakeSys/ros2/src/snake/snake_gui/include/snake_gui/qcustomplot.h` and `SnakeSys/ros2/src/snake/snake_gui/src/qcustomplot.cpp` are licensed under the [GNU GPL](https://www.gnu.org/licenses/gpl.html) except as noted otherwise.
3. Source codes in folder `SnakeSys/ros2/src/sensor_camera/realsense_cam` are under the [Apache License, Version 2.0](http://www.apache.org/licenses/LICENSE-2.0).
4. Source codes in folder `SnakeSys/ros2/src/sensor_camera/zed_cam` are under the [Apache License, Version 2.0](http://www.apache.org/licenses/LICENSE-2.0).
5. Source codes in folder `SnakeSys/ros2/src/sensor_camera/usb_cam` are under the [BSD license](https://github.com/ros-drivers/usb_cam/blob/ros2/LICENSE).
6. Source codes in folder `SnakeSys/ORB_SLAM3` are under the [GPLv3 license](https://github.com/UZ-SLAMLab/ORB_SLAM3/LICENSE), and for a list of all code/library dependencies (and associated licenses), please see [Dependencies.md](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/Dependencies.md).
7. Library dependencies:
  - `Qt 6.7.0` is under [Qt Licensing](https://www.qt.io/qt-licensing).
  - `Boost 1.85.0` is under the [Boost Software License](https://www.boost.org/LICENSE_1_0.txt).
  - `GSL 2.7.1` is under the [GNU General Public License](https://www.gnu.org/licenses/gpl.html) (GPL).
  - `Eigen 3.4.0` is [Free Software](http://www.gnu.org/philosophy/free-sw.html). Starting from the 3.1.1 version, it is licensed under the [MPL2](https://www.mozilla.org/en-US/MPL/2.0/), which is a simple weak copyleft license. Common questions about the MPL2 are answered in the official [MPL2 FAQ](https://www.mozilla.org/en-US/MPL/2.0/FAQ/). Earlier versions were licensed under the LGPL3+.
  - `fmt 10.2.1` is under the [MIT license](https://github.com/fmtlib/fmt/blob/master/LICENSE).
  - `Sophus 1.22.10` is under the [MIT license](https://github.com/strasdat/Sophus/blob/main/LICENSE.txt).
  - `VTK 9.3.0` is under the [BSD license](http://en.wikipedia.org/wiki/BSD_licenses).
  - `OpenCV 4.9.0`. `OpenCV 4.5.0` and higher versions are licensed under the [Apache 2 License](https://github.com/opencv/opencv/blob/master/LICENSE). `OpenCV 4.4.0` and lower versions, including `OpenCV 3.x`, `OpenCV 2.x`, and `OpenCV 1.x`, are licensed under the [3-clause BSD license](https://github.com/opencv/opencv/blob/4.4.0/LICENSE).
  - `PCL 1.14.1` is under the [BSD license](https://github.com/PointCloudLibrary/pcl/blob/master/LICENSE.txt).

------

