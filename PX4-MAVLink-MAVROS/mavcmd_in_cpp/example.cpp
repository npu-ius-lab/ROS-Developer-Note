/***
展示如何在主程序中发送提高话题频率的 **代码片段** 。
使用时，只需要包含必要的头文件，然后插入到任何你需要的地方即可。
***/

#include <ros/ros.h>
#include <mavros_msgs/CommandLong.h>

cmd_client_ = nh_.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");
cmd_client_.waitForExistence(ros::Duration(3.0));
mavros_msgs::CommandLong cmd_msg;
cmd_msg.request.broadcast = false;                      // 不广播
cmd_msg.request.confirmation = false;                   // 不需要确认
cmd_msg.request.command = 511;                          // MAV_CMD_SET_MESSAGE_INTERVAL（设定特定MAVLink消息）
cmd_msg.request.param1 = 32;                            // 参数1：LOCAL_POSITION_NED （局部位置消息ID：32）
// 若设定IMU频率，则cmd_msg.request.command = 105; 
double hz = 50.0;
cmd_msg.request.param2 = 1000000.0 / hz;                // 参数2：频率50Hz
cmd_client_.call(cmd_msg);
ROS_INFO_ONCE("已请求提高消息发布频率至50Hz。");
