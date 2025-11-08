# MAVROS和PX4的坐标系差异

> 作者：张子宇

MAVROS是PX4飞控和ROS系统通信的桥梁。不论是获取飞行器的数据，还是向飞行器发送指令，都需要统一坐标系，尤其是在使用GPS定位，或者需要直接控制无人机的姿态，或者需要自主设计规划/控制程序时。

## 结论
1. **具有全局定位（GPS）和全局航向（磁罗盘）时，MAVROS的世界坐标系为：ENU（东北天），MAVROS的机体坐标系为：FLU（前左上）**
>世界的x轴正向为东，y轴正向为北，z轴正向指天
>
>机体的x轴正向指向机头，y轴正向指向左侧，z轴正向垂直于机身指向上方
>
>三轴角度遵循右手定则：俯仰角低头为正、抬头为负；滚转角右滚为正、左滚为负；偏航角以正东为0，逆时针增大，范围(-pi,pi)。

2. **只具有局部定位（VIO等）时，MAVROS的世界坐标系为：FLU（前左上），原点位于初始化点**
>以接入VIO时的位姿为原点，前、左、上指向分别为世界x,y,z轴指向，建立世界坐标系
>
>机体坐标系定义不变

## 与PX4的差异
PX4内部采用的是全局NED（北东地）和局部FRD（前右下）坐标系，这和ROS、MAVROS的约定是完全相反的。
幸运的是，MAVROS已经帮我们处理好了坐标转换：无论是从MAVROS拿数据，还是向MAVROS发数据，我们都只需要遵循结论中ROS的标准坐标系定义即可。

## 特别提醒
当前版本的MAVROS（至少在ROS Noetic，即Ubuntu 20.04中）的**发送坐标点指令**，枚举类型的**名称与实际不符**。例如，下面的代码将要向`/mavros/setpoint_raw/local`话题中发送指令：
```cpp
mavros_msgs::PositionTarget local_pos_target;

local_pos_target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
// 务必注意这里的 FRAME_LOCAL_NED ！！
local_pos_target.header.stamp = ros::Time::now();
local_pos_target.position.x = 1.0;
local_pos_target.position.y = 1.0;
local_pos_target.position.z = 1.0;
local_pos_target.yaw_rate = 0.0;
local_pos_target.type_mask = mavros_msgs::PositionTarget::IGNORE_VX | mavros_msgs::PositionTarget::IGNORE_VY | mavros_msgs::PositionTarget::IGNORE_VZ |
mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ |
mavros_msgs::PositionTarget::IGNORE_YAW;
```

这个`FRAME_LOCAL_NED`标识可能会迷惑我们：我们将要前往的坐标点是NED坐标系下的`(1.0, 1.0, 1.0)`点，但事实上并不是！**这个点是ENU系下的！！！**
同样，如果你使用`FRAME_BODY_NED`，指令点则定义在**FLU机体坐标系下**，而不是FRD！
>**为什么是这样？**
>如果你已经理解了MAVLink和MAVROS的关系，那么就知道，MAVROS的话题/消息内容实际上就是对MAVLink定义的消息的重新包装。MAVLink中定义了很多坐标系枚举标志，但MAVROS的开发者偏偏选用了`NED`的标志，可能是因为他们觉得，发送去PX4的命令最终是NED下的。
>
>但这会迷惑我们开发者！我们只需要关心：发送到MAVROS的指令是什么坐标系就可以了。


## 测试
测试xyz坐标定义很容易，但三轴角度则不易直接观察（因为消息中是四元数）。下面给出一段测试三轴角度定义的python脚本，在终端运行后，你可以实时看到三轴角度的数值。
欧拉角的旋转顺序遵循tf库的定义（似乎是Z-Y-X？）。
```python
#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
import math
import sys

def pose_callback(msg):
"""
回调函数，用于处理接收到的位姿信息
"""
# 从PoseStamped消息中提取四元数
orientation_q = msg.pose.orientation
orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
# 将四元数转换为欧拉角
(roll, pitch, yaw) = euler_from_quaternion(orientation_list)
# 将弧度转换为度
roll_deg = math.degrees(roll)
pitch_deg = math.degrees(pitch)
yaw_deg = math.degrees(yaw)

display_str = f"滚转角(Roll): {roll_deg:7.2f}° 俯仰角(Pitch): {pitch_deg:7.2f}° 偏航角(Yaw): {yaw_deg:7.2f}°"
# 在终端的同一行实时显示所有角度
# '\r' 将光标移动到行首，允许下一次打印覆盖本次内容
sys.stdout.write('\r' + display_str + ' ' * 5)
sys.stdout.flush()

def main():
"""
主函数
"""
# 初始化ROS节点
rospy.init_node('mavros_yaw_display_node', anonymous=True)
# 订阅 /mavros/local_position/pose 话题
rospy.Subscriber("/mavros/local_position/pose", PoseStamped, pose_callback)
rospy.loginfo("正在从 /mavros/local_position/pose 话题获取偏航角...")
# 保持程序运行，直到ROS关闭
rospy.spin()


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
	finally:
		# 确保程序退出时，终端能换行
		print("\n程序已退出。")
```

## 参考文献
详见：
[The doc of mavros_msgs about coordinate needs better clarification](https://github.com/mavlink/mavros/issues/1500)
[mavros坐标系转换与方向](https://blog.csdn.net/benchuspx/article/details/112970682)