# Model Predictive Control for Bipedal Robot PF_TRON1A in MuJoCo Simulator
# 双足机器人PF_TRON1A在MuJoCo仿真器中的模型预测控制

![video](./display.gif)

This work is focused on the study of convex MPC for bipedal robots.
Convex model predictive control can be applied not only to quadruped robots but also to biped robots.

本项目专注于双足机器人的凸优化模型预测控制研究。
凸模型预测控制不仅可以应用于四足机器人，也可以应用于双足机器人。

# Important matter / 重要事项
**This project is currently running only on ros2！！**
**该项目目前只能在ROS2上运行！！**

**Please check the ros versions supported by your operating system！！**
**请检查您的操作系统支持的ROS版本！！**

# Build Package / 构建包
## Install ROS2 according to your operating system (my operating system is ubuntu 22.04 with ros2-humble)
## 根据您的操作系统安装ROS2（我的操作系统是ubuntu 22.04，使用ros2-humble）

If you are freshman, it is recommended to use this command line in terminal:
如果您是新手，建议在终端中使用以下命令：
```bash
wget http://fishros.com/install -O fishros && . fishros
```
First, choose 1: '一键安装(推荐):ROS(支持ROS/ROS2,树莓派Jetson)'.
首先，选择1：'一键安装(推荐):ROS(支持ROS/ROS2,树莓派Jetson)'。

Second, choose 1: '更换系统源再继续安装'.
其次，选择1：'更换系统源再继续安装'。

Third, choose 1: 'humble(ros2)'.
第三，选择1：'humble(ros2)'。

Finally, choose 1: 'humble(ros2)桌面版'.
最后，选择1：'humble(ros2)桌面版'。

Then, the ros2 is successfully installed. For more details please refer to [fishros](https://github.com/fishros/install). Thank fishros for his contributions!!!!!!!
然后，ROS2就成功安装了。更多详情请参考[fishros](https://github.com/fishros/install)。感谢fishros的贡献！！！！！！

If ros2 cannot be successfully installed in your operating system, please refer to [ROS2](https://ros.org/) official doc.
如果在您的操作系统中无法成功安装ROS2，请参考[ROS2](https://ros.org/)官方文档。

## Dependency / 依赖项
```bash
sudo apt update
sudo apt-get install ros-<ros2-distro>-eigenpy
sudo apt-get install ros-<ros2-distro>-pinocchio
sudo apt install libglfw3-dev
```

For ros2(humble) / 对于ros2(humble)：
```bash
sudo apt update
sudo apt-get install ros-humble-eigenpy
sudo apt-get install ros-humble-pinocchio
sudo apt install libglfw3-dev
```

## Build / 构建
```bash
git clone https://github.com/81578823/Bipedal_MPC.git
```

```bash
cd Bipedal_MPC & bash build.sh
echo "source ~/Bipedal_MPC/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

# Run Package / 运行包
## Run Simulation / 运行仿真
```bash
ros2 launch sim sim_launch.py 
```

## Run Controller / 运行控制器
```bash
ros2 launch management management_launch.py 
```

# Change Velocity / 改变速度
if don't have the remote controller, you can change the linear velocity and angular velocity on this code file:
如果没有遥控器，您可以在此代码文件中更改线速度和角速度：
![photo](./velocity_get.jpg)

Then you can change the velocities on the JoyStick::getLinearVelCmd() and JoyStick::getYawVelCmd() (default: 0.0 (linear); 0.0 (angular))
然后您可以在JoyStick::getLinearVelCmd()和JoyStick::getYawVelCmd()中更改速度（默认值：0.0（线速度）；0.0（角速度））

# Future work / 未来工作
Nowadays, more and more people are used to using python. 
Hence, we will focus on converting the C++ work into a respository in python. So please waiting ...

如今，越来越多的人习惯使用Python。
因此，我们将专注于将C++工作转换为Python存储库。所以请稍等...

# Reference / 参考文献
J. Di Carlo, P. M. Wensing, B. Katz, G. Bledt and S. Kim, "Dynamic Locomotion in the MIT Cheetah 3 Through Convex Model-Predictive Control," 2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Madrid, Spain, 2018, pp. 1-9, doi: 10.1109/IROS.2018.8594448. keywords: {Robot kinematics;Legged locomotion;Dynamics;Predictive control;Convex functions;Predictive models},




