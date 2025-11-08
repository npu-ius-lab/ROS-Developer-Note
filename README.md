# ROS & PX4 开发笔记

> 记录作者在ROS与PX4开发过程中：
> - 实现的工具、便捷方法
> - 代码规范、开发指南
> - 总结的经验、问题与解决方法
> - 软件、依赖的配置方法

> 如无特别说明，开发环境均为 **Ubuntu 20.04 + ROS Noetic**。 
> 持续更新，仅供参考。

---
## 规范
推荐使用Obisidian笔记软件记录。它基于Markdown格式，易于编辑，界面美观，第三方插件丰富，能与Github集成，从而提高协作、同步能力。
Obsidian软件在Windows下应当是开箱即用的。如果在Ubuntu 20.04中使用，因为Electron框架的更新，需要下载历史版本的Appimage。详情查看[这里](Software/obsidian/obsidian.md)。

如果要对该项目进行贡献，请务必：
1. 为笔记单独建立文件夹，并适当归类；
2. 遵守标准的Markdown语法规范；
3. 在目录中指向你的笔记内容。

## 目录

### PX4 & MAVLink & MAVROS 相关
1. [`about_px4.launch`](PX4-MAVLink-MAVROS/about_px4.launch/about_px4.launch.md)
> 记录了利用MAVROS连接飞控的原理和参数配置方法，以及利用MAVROS，**使用udp、tcp协议桥接QGC或其他MAVLink客户端**的方法。

2. [`mavcmd_in_cpp`](mavcmd_in_cpp.md)
> 记录了在C++程序代码中使用MAVROS提供的rosservice向飞控发送MAVLink命令的方法，以**程序化地提高IMU频率、位姿话题频率、强制解锁、设置参数**等

3. [`reset_ekf2`](reset_ekf2.md)
> 分析了PX4连接GPS时，`local_position`话题的z轴高度出现大幅度漂移的问题，并给出了不下电、**程序化重启EKF位姿估计模块**的方法

4. [`setpoint_frame`](PX4-MAVLink-MAVROS/setpoint_frame/setpoint_frame.md)
> 分析了通过MAVROS发送setpoint指令时，容易误解的**坐标系规范**。

### ROS 相关
1. `easy-configure`
> 开发了一种利用python生成脚本，在C++编程的ROS程序中，**一次性、集中地书写、读取并加载全部参数**的方法，避免了添加参数时，需要在修改yaml的同时在程序中添加getParam的麻烦。

2. `SSH-connect`
> 记录了在低性能的机载计算机上，如何建立**无图形界面的SSH终端连接**。同时还记录了不通过ip而是主机名来解析目标主机的方法，在频繁切换WIFI的环境中可能有用。

### 硬件相关

1. [`usb-cam-connect`](Hardware/usb-cam-connect/usb-cam-connect.md)
> 记录了**使用OpenCV连接USB相机并获得画面**的方法。此外还记载了**由ROS图像话题、甚至任意视频、图像来虚拟USB相机设备**的方法。

### 软件相关

1. `obsidian`
> 记录了**Ubuntu 20.04系统中安装obsidian笔记软件**的方法。

2. `git-learning`
> 记录了**学习git相关命令**的笔记。

3. `libtorch-and-trt`
> 记录了：  
> (1)X86计算机、Jetson-NX系列**部署libtorch**(pytorch的C++API)的方法；  
> (2)如何将pytorch的模型**转为torchscript**以在python无关的环境中调用和推理；  
> (3)将torchscript模型**用TensorRT加速**推理的同时不更改代码和API接口。
