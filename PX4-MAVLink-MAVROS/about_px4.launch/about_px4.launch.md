# MAVROS的px4.launch文件通信配置说明

> 作者：张子宇

MAVROS实际上是PX4与ROS之间的一个通信“桥梁”，这个桥梁建立在MAVLink协议上。当然作为开发者，我们无需关心这是如何实现的，只需要知道一些实用的技巧即可。

关于**编程地自动提高IMU话题频率**的问题，请查看[这里](/PX4-MAVLink-MAVROS/mavcmd_in_cpp/mavcmd_in_cpp.md)。

关于**编程地重置`local_position`系列话题原点**的问题，请查看[这里](/PX4-MAVLink-MAVROS/reset_ekf2/reset_ekf2.md)。

关于**MAVROS发送坐标指令的坐标系问题**，请查看[这里](/PX4-MAVLink-MAVROS/setpoint_frame/setpoint_frame.md)。

关于MAVROS和PX4的**通信配置问题**，请查看本文。

## 基本原理

从通信的角度讲，MAVROS的核心任务是在飞控（通过串口/USB）和一个或多个地面站/客户端（通过网络）之间双向转发MAVLink消息。我们可以打开MAVROS的px4.launch文件来查看：
> 如果使用`apt`安装，则它一般在`/opt/ros/noetic/share/mavros/launch`路径下。当然你也可以使用`roscd mavros`来查看。

![[PX4-MAVLink-MAVROS/about_px4.launch/img/1.png]]
其中最重要的两个参数为：`fcu_url`和`gcs_url`，指示了MAVROS与物理串口的上行连接地址，以及MAVROS与QGC等客户端桥接的下行连接地址。

## 物理串口连接

`fcu_url`是MAVROS用来连接到**飞控**的地址。它通常是一个串口设备，如 /dev/ttyACM0:921600。MAVROS会**独占**这个连接，这是它与飞控通信的唯一通道。
> 因此，当你通过MAVROS连接PX4后，又打开本机上了QGC（如果保持了默认设置），那么MAVROS就会被QGC挤占下线。因为物理串口连接是**独占**的，而QGC在默认情况下会尝试用串口直连飞控。

通常来说，我们用串口线（RX/TX）连接飞控和机载电脑时，识别到的串口地址一般为`/dev/ttyUSB0`或者`/dev/ttyTHS0`，此时设置`fcu_url`时就需要在后面设置一个正确的通信波特率，格式为：
```
"/dev/ttyUSB0:921600"
```
这代表：MAVROS和飞控使用`/dev/ttyUSB0`建立连接，波特率是921600。

此外，我们还可以使用USB线直连PX4和飞控，这样识别到的串口地址就为`/dev/ttyACM0`。ACM是一种通过USB虚拟串口的技术，对它来说设置波特率是无意义的，因为USB的通信速率远高于串口。但是你仍然需要设置一个值来满足语法条件，所以仍然书写921600吧。

个人推荐使用USB线直连的方法连接（虽然有时它对机身空间要求较高）。但是注意，[Ubuntu内置的调制解调器模块可能会影响通信](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html)，需要卸载：
```bash
sudo apt remove --purge modemmanager
```

此外，上述官方文档中也给出了**无需授权或sudo与串口设备通信的方法**，即将用户添加到 dialout 组：
```bash
sudo usermod -aG dialout "$(id -un)"
```

这条命令实质上解释了为什么有些电脑连接MAVROS可以不用`sudo chmod 777 /dev/ttyUSB0`来赋权限，而有些电脑需要。此外，执行了上述命令，就**再也不需要为串口单独赋权了**，原理详见[这里](about_usermod.md)。
## QGC桥接

`gcs_url`是MAVROS提供的，用来桥接到**QGC等客户端**的地址。因为MAVROS独占了物理通信端口，导致QGC等客户端没有端口能与飞控通信，所以MAVROS提供了这个功能。

一般来说，该地址都设为使用udp或tcp协议。MAVROS会自动在你指定的通信端口上转发。

> 理论上来讲，如果你给飞控和机载电脑之间接两根线，那么`fcu_url`和`gcs_url`都可以设为物理串口地址（但我没试过这样做，感觉也很怪）。

> 理论上来讲，`fcu_url`也可以使用udp或tcp通信，但这就更怪了，一般只会在仿真环境中使用。

`gcs_url`的语法如下：

1. **udp连接**
	 通用格式如下：
	 ```
	 udp://[bind_host][:port]@[remote_host][:port][/?ids=sysid,compid]
	 ```
	 各部分含义：
	 - `udp://`：指示这是一个UDP连接。
	 - `[bind_host][:port]`(例如`0.0.0.0:14555`)：表示MAVROS监听的IP和端口。IP一般留空（即`0.0.0.0`），意为监听所有IP地址；端口默认是14555。
	 > **监听**指的是MAVROS监听**来自地面端的指令**，例如QGC发送的上锁、解锁、切换模式指令；或者[自行编写的脚本发送的MAVLink指令](PX4-MAVLink-MAVROS/reset_ekf2/reset_ekf2.md)。
	 
	 - `[remote_host][:port]`(例如`127.0.0.1:14550`)：表示MAVROS发送数据的IP和端口。IP指定为`127.0.0.1`或`localhost`表示本机环回；端口默认是14550。
	 > **发送**数据指的是MAVROS将从飞控获得的数据发送到需要显示的地面端，例如QGC。事实上我们可以看出，UDP连接的发送和接收需要分别在两个端口上建立。
	 
	 - `[/?ids=sysid,compid]`：可选，用于指定系统id。
	 
	 广播格式如下：
	 ```
	 udp-b://[bind_host][:port]@[:port][/?ids=sysid,compid]
	 ```
	 这种模式下，MAVROS 会在初始阶段进行 UDP 广播，直到发现地面站后，切换到该地面站的地址进行单播通信。
	 
	 永久广播格式如下：
	```
	udp-pb://[bind_host][:port]@[:port][/?ids=sysid,compid]
	```
	在这种模式下，MAVROS 会持续进行 UDP 广播通信。
	
	通常我们会这样书写UDP连接：
	```
	udp://:14550@192.168.1.100:14555
	```
	这会让：
	(1) MAVROS接收本地所有ip地址的14550端口上发送的数据（即使不是QGC发送的，例如自行编写的MAVLink脚本）
	(2) 将所有需要传回的数据发送到192.168.1.100的14555端口（你的QGC应该运行在这个ip和端口上）
---
2.  **tcp连接**
	TCP客户端格式：
	```
	tcp://[server_host][:port][/?ids=sysid,compid]
	```
	-  `tcp://`：指示MAVROS是TCP客户端。
	- `[server_host]`：地面站（作为 TCP 服务器）的 IP 地址，默认为 localhost。
	- `[:port]`：地面站的 TCP 端口，默认为 5760。
	- `[/?ids=sysid,compid]`：可选，用于指定系统id。
	> 通常我们不采用这种连接方式，因为QGC只能应该作为TCP客户端（去请求服务端的连接），因此MAVROS需要作为服务端。
	
	TCP服务端格式：
	```
	tcp-l://[bind_host][:port][/?ids=sysid,compid]
	```
	-  `tcp-l://`：指示MAVROS是TCP服务端（`-l`表示listen，意为服务端正在监听来自指定端口上客户端的请求）。
	- `[server_host]`：MAVROS 监听端口的 IP 地址，默认为 0.0.0.0，即监听所有IP上的端口。
	- `[:port]`：MAVROS 监听的TCP端口，默认为 5760。
	- `[/?ids=sysid,compid]`：可选，用于指定系统id。
	> MAVROS 会作为 TCP 服务器，等待地面站的连接。
	
	通常我们会这样书写TCP连接：
	```
	"tcp-l://:5760"
	```
	这会让：
	MAVROS接收本地所有ip地址的5760端口上建立TCP连接的请求。
	> 这意味着，你可以通过不只一个客户端与PX4通信。例如说，QGC通过192.168.1.1:5760端口连接，而你的[MAVLink Shell脚本](PX4-MAVLink-MAVROS/reset_ekf2/reset_ekf2.md)通过127.0.0.1:5760端口发送所需指令。

## 其他说明

从通信速率上来说，我建议配置TCP连接，它更快、更稳定。但是TCP连接无法被QGC自动发现，需要手动配置连接。

我们既可以在启动px4.launch时传递ROS参数来设置`fcu_url`和`gcs_url`，也可以直接`sudo gedit px4.launch`修改。