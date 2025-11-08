# 编程地重置PX4的EKF2原点，刷新`local_position`位置

> 作者：张子宇

## 问题来源

在开发某项目的实验中，观察到如下问题：

1. **使用GPS定位时**，`/mavros/local_position/pose`等话题的z轴高度不准确，甚至出现-600m的数值
2. 飞控能够稳定飞行，高度的变化趋势、变化率是稳定的，只是数值不正确
3. 查看`/mavros/altitude`话题，发现local高度和`/mavros/local_position/pose`给出的高度一致，而relative高度是基本正确的

查阅官方文档与[社区讨论](https://github.com/PX4/PX4-Autopilot/issues/21805)发现，上述问题基本是由于**EKF2原点初始化过早引起的**。

`/mavros/local_position/pose`话题给出的是PX4的local坐标系下的位置（同样可以在QGC的MAVLINK检测的LOCAL_POSITION_NED中看见，二者数值一致，只是坐标定义相反）。这个local系的原点是PX4上电的初始位置。

> 另外一提，[MAVROS的局部/世界坐标系是ENU、机体坐标系是FLU，这与ROS标准完全一致](PX4-MAVLink-MAVROS/setpoint_frame/setpoint_frame.md)；PX4的局部/世界坐标系是NED、机体坐标系是FRD。MAVROS内部已经处理了这种转换，因此我们在给MAVROS发送数据/从MAVROS拿数据时，都在ENU/FLU下。

这导致：

1. PX4上电时，GPS同时启动。它在启动之初的高度定位可能不准确，但被EKF2采信了，并以此初始化了原点。
2. 稍后，GPS信号稳定时，可能正确地给出了当前高度，这个高度与上述原点的高度有较大差异，于是导致z轴高度数值错误。
3. 对地高度是实时更新的，这可以通过观察`/mavros/global_position/local`的坐标位置来观察。

> 参见[mavros中的一些坑](https://blog.csdn.net/kenhuyang/article/details/85020865)，[mavros和PX4中的海拔高与椭球高转换](https://blog.csdn.net/benchuspx/article/details/135854089)

## 解决思路

1. 启动时，[待GPS稳定后手动从QGC控制台重启EKF2](https://discuss.px4.io/t/programmatically-reset-the-ekf/10484)
> 这需要能够使用QGC，而且比较麻烦。

2. 订阅`/mavros/altitude`或者`/mavros/global_position/local`获得对地高度，并在代码中用该值补偿
> 实现上难度不大，但前一个话题频率只有10Hz，后一个话题的具体消息还有待观察

3. 为GPS独立供电并提前启动
> 我感觉没有必要。同时这也指出：直接重启飞控是没有用的：因为GPS仍然会重新上电。

思路2和3能够解决问题，但都没有直面问题的根源：**EKF2原点不正确。**
思路1虽然解决了问题的根源，但它依赖于QGC，启动很麻烦。

## 最终方案

我希望设计一个编程化重启EKF2的方案。查阅PX4官方文档发现：
1. **EKF2是[PX4的一个模块](https://docs.px4.io/main/en/modules/modules_estimator#ekf2)**
2. **PX4的模块可以通过[PX4自带的Shell](https://docs.px4.io/main/en/debug/consoles#using_the_console)来操作**
3. PX4**通过MAVLink协议暴露了[外部访问PX4 Shell的接口](https://docs.px4.io/main/en/debug/mavlink_shell)**（称作MAVLink Shell）
4. PX4给出了通过Python脚本访问MAVLink Shell的方法

据此便有了解决思路：将需要发送的命令写进[Python脚本](reset_ekf2.py)中并发送。具体实现请查阅脚本，这里不再展开实现原理。
> 注意需要安装python相关的依赖，因为本质上它是基于3中官方文档给出的脚本改编的。需要执行：`pip install pymavlink pyserial`

> 该脚本期望一个输入参数，指定脚本通过什么方式与PX4建立通信连接。一般来说，在MAVROS占用物理串口的情况下，都通过MAVROS提供的桥接连接。详见[about_px4.launch](PX4-MAVLink-MAVROS/about_px4.launch/about_px4.launch.md)。

从理论上来说，上述程序完全可以在C++中实现，因为MAVLink提供了C++的API。待更新。