# USB相机的连接与驱动方法

> 作者：张子宇

在ROS上开发有很多好处：我们无需关心硬件的驱动如何实现，通过话题机制就可以从设备上拿到所需的数据。

当然前提是：需要为设备开发一个ROS节点，从驱动拿数据并发布为ROS话题（代价总要有人来承担，对吧？）。

对于USB相机，我们常用的方法是使用`usb_cam`这个功能包。在某些应用中，通过ROS节点转发相机的数据是不必要、不经济的：
- 转发会引入额外的通信开销和延迟；
- 需要额外启动一个ROS节点，并保证它不能挂掉；
- 需要额外配置它的参数文件，且不能定制化。
- 对于目标识别等应用，本身就要发布调试图像，这又会造成信息的重复。

那么为什么不能**直接在程序中获取到USB相机的数据**呢？

## 在Python中测试

非常简单，使用OpenCV即可。这是因为，Linux集成了通用的相机驱动后端Video4Linux2 (V4L2)来与摄像头交互，而OpenCV即可直接调用它。

在插入USB相机前后，终端中执行下列命令：
```bash
ls /dev/video*
```
你会发现，插入相机后有更多的设备被list出来了，例如`/dev/video2`。请记下这个数字。
> 有时，一个USB相机可能会产生多个`/dev/video*`，详细解释请看后文。

在Python中，我们可以这样加载相机：
```python
camera_index = 2
cap = cv2.VideoCapture(camera_index)
```
这样就会将`/dev/video2`对应的相机加载到`cap`变量中。要读取相机的画面，只需要：
```python
ret, frame = cap.read()
if not ret:
	print("错误：无法读取视频帧。")

window_name = 'Resizable Camera Feed'
cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
cv2.imshow(window_name, frame)
```
可以使用[这个脚本](show_cam.py)来查看相机的实时画面。

## 在C++中调用

显然上述流程可以集成到C++中，因为C++也有OpenCV的API。

只需要在你的代码中定义一个`cv::VideoCapture`类型的全局变量`cap_`，就可以通过如下代码来初始化这个相机了：
```cpp
int camera_index = 2;
cap_ = cv::VideoCapture(camera_index, cv::CAP_V4L2);
if (!cap_.isOpened()){ 
	std::cout << "错误：无法加载设备" << std::endl;
}
```
读取画面的API和Python的一样。如果我们要持续读取画面，则可以在循环中进行：
```cpp
while (true) {
    cv::Mat frame;
    // 从摄像头抓取一帧图像
    bool is_success = cap.read(frame); 

    if (!is_success || frame.empty()) {
        ROS_WARN("Failed to grab frame. Is the camera disconnected?");
        continue;
    }
}
```
但是这样做不太符合我们的ROS实践规范：所有循环调用的进程应该都写在单独的回调函数（Callback）中，由`ros::spin`统一管理。因此我们应该为它创建一个定时器（Timer）。
> 实际上就是取代了以前订阅ROS话题的`image_callback`。

示例C++实现参见[这里](show_cam.cpp)。
> 这段程序还智能地处理了相机帧率低于定时回调函数频率的问题：
> 
> `cap.grab()`只从摄像头缓冲区抓取一帧原始数据到 VideoCapture 对象的内部缓存中，这是一个非常快速的操作。如果缓冲区中**有新的一帧**，它返回`true`。如果没有，返回`false`。
> 
> 当有新帧时，`cap.retrieve()`将内部缓存中的数据解码，存入你提供的 cv::Mat 对象中。

我们可以看到，用OpenCV方法得到的画面原生就是`cv::Mat`类型的，它不再需要cv_bridge的转换。

如果出于某些原因（例如，需要录制rosbag），也可以再借助cv_bridge将捕获到的画面转换为ROS消息并发布，这和使用`usb_cam`功能包的原理是一致的，但避免了单独启动节点的麻烦，还给予你更大的程序设计自由度，并提供了与ROS解耦的可能。

## 进阶调用方法

上面的C++示例同样也给出了一些进阶调用方法，即**设置相机的参数**。注意，OpenCV的方法只能*尝试*设置这些参数，它不保证能成功。为了查看设备能够支持哪些参数，我们可以在终端中运行：
```bash
sudo apt-get update
sudo apt-get install v4l-utils
```
安装上述工具后，执行：
```bash
v4l2-ctl -d /dev/video2 --list-formats-ext
```
根据你的设备索引更改数字。你应该可以看到输出中有如下的行：
（TODO）

这标识了你的设备支持什么样的分辨率和帧率组合。你可以根据上述的组合，在代码中用如下方式来设置：
```cpp
cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
cap_.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
cap_.set(cv::CAP_PROP_FPS, 30);
```
> 一般来说，YUV格式的图像帧率远低于MJPG格式，因此`cv::CAP_PROP_FOURCC`字段的设置是不能省略的，否则OpenCV可能会加载为YUV格式，导致视频流非常卡顿。

此外我们还可以更改其他设置，例如亮度、对比度等。执行：
```bash
v4l2-ctl -d /dev/video2 --all
```
根据你的设备索引更改数字。你应该可以看到输出中有如下的行：
```bash
brightness 0x00980900 (int) : min=-64 max=64 step=1 default=0 value=64  
contrast 0x00980901 (int) : min=0 max=95 step=1 default=0 value=0  
saturation 0x00980902 (int) : min=0 max=100 step=1 default=56 value=56  
hue 0x00980903 (int) : min=-2000 max=2000 step=1 default=0 value=0  
white_balance_temperature_auto 0x0098090c (bool) : default=1 value=1  
gamma 0x00980910 (int) : min=100 max=300 step=1 default=115 value=115  
gain 0x00980913 (int) : min=0 max=255 step=1 default=120 value=120  
power_line_frequency 0x00980918 (menu) : min=0 max=3 default=1 value=1  
0: Disabled  
1: 50 Hz  
2: 60 Hz  
3: Auto  
white_balance_temperature 0x0098091a (int) : min=2800 max=6500 step=1 default=4600 value=4600 flags=inactive  
sharpness 0x0098091b (int) : min=0 max=7 step=1 default=0 value=0  
backlight_compensation 0x0098091c (int) : min=0 max=100 step=1 default=50 value=50  
exposure_auto 0x009a0901 (menu) : min=0 max=3 default=3 value=1  
1: Manual Mode  
3: Aperture Priority Mode  
exposure_absolute 0x009a0902 (int) : min=1 max=10000 step=1 default=156 value=156
```
这标识你的相机支持哪些参数，参数的设置范围(`min,max`)，默认值(`default`)，设置步长(`step`)和当前值(`value`)。你可以根据上述内容，在代码中调用OpenCV的`set`方法来设置相关参数：

| `v4l2-ctl` 属性名称                      | OpenCV 属性 (`cv2.CAP_PROP_*`)      | 含义与解释                                   | 如何在OpenCV中设置与注意事项                                                                                           |
| :----------------------------------- | :-------------------------------- | :-------------------------------------- | :---------------------------------------------------------------------------------------------------------- |
| **`brightness`**                     | `cv2.CAP_PROP_BRIGHTNESS`         | 图像的整体亮度。                                | `cap.set(cv2.CAP_PROP_BRIGHTNESS, value)`。注意OpenCV的范围可能与v4l2不同（常被归一化到0-255），最好先`get()`一下当前值来判断范围。           |
| **`contrast`**                       | `cv2.CAP_PROP_CONTRAST`           | 图像最亮和最暗区域的差异。高对比度使图像看起来更“鲜明”。           | `cap.set(cv2.CAP_PROP_CONTRAST, value)`                                                                     |
| **`saturation`**                     | `cv2.CAP_PROP_SATURATION`         | 颜色的鲜艳程度。0为黑白图像，值越高颜色越饱和。                | `cap.set(cv2.CAP_PROP_SATURATION, value)`                                                                   |
| **`hue`**                            | `cv2.CAP_PROP_HUE`                | 图像的色调，整体颜色偏移。通常保持默认值0。                  | `cap.set(cv2.CAP_PROP_HUE, value)`                                                                          |
| **`white_balance_temperature_auto`** | `cv2.CAP_PROP_AUTO_WB`            | 是否自动调整白平衡。白平衡确保白色物体在图像中显示为白色。           | `cap.set(cv2.CAP_PROP_AUTO_WB, 1)` 为开启，`0` 为关闭。                                                             |
| **`gamma`**                          | `cv2.CAP_PROP_GAMMA`              | 伽马校正。调整图像中间色调的亮度，影响图像的“深度感”。            | `cap.set(cv2.CAP_PROP_GAMMA, value)`                                                                        |
| **`gain`**                           | `cv2.CAP_PROP_GAIN`               | 图像传感器的信号增益。在低光下增加此值可提亮画面，但会显著增加噪点。      | `cap.set(cv2.CAP_PROP_GAIN, value)`                                                                         |
| **`power_line_frequency`**           | `cv2.CAP_PROP_SETTINGS` (通常不直接映射) | 防止室内交流电灯光（50Hz或60Hz）在画面中产生闪烁条纹。         | **这个通常无法通过一个简单的 `set` 来调整**。OpenCV的V4L2后端通常会尝试自动处理。如果必须设置，你可能需要使用系统命令 `v4l2-ctl -c power_line_frequency=1`。 |
| **`white_balance_temperature`**      | `cv2.CAP_PROP_WB_TEMPERATURE`     | 手动设置白平衡色温值（单位：开尔文）。低值偏暖（黄），高值偏冷（蓝）。     | 必须先 `cap.set(cv2.CAP_PROP_AUTO_WB, 0)` 关闭自动白平衡。然后 `cap.set(cv2.CAP_PROP_WB_TEMPERATURE, 4600)`。             |
| **`sharpness`**                      | `cv2.CAP_PROP_SHARPNESS`          | 图像锐度。通过算法增强边缘，使图像看起来更“清晰”，但过度会产生不自然的白边。 | `cap.set(cv2.CAP_PROP_SHARPNESS, value)`                                                                    |
| **`backlight_compensation`**         | `cv2.CAP_PROP_BACKLIGHT`          | 背光补偿。当背景很亮而主体很暗时，此功能会提亮整个画面，以看清主体。      | `cap.set(cv2.CAP_PROP_BACKLIGHT, value)`                                                                    |
| **`exposure_auto`**                  | `cv2.CAP_PROP_AUTO_EXPOSURE`      | 曝光模式。你的相机支持手动和光圈优先模式。                   | `cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)` 设为手动模式。`cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)` 设为自动模式。            |
| **`exposure_absolute`**              | `cv2.CAP_PROP_EXPOSURE`           | 手动设置绝对曝光时间。值越大，进光量越多，画面越亮，但也越容易因运动而模糊。  | 必须先 `cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)` (或 0.25) 设为手动曝光。然后 `cap.set(cv2.CAP_PROP_EXPOSURE, value)`。    |
当然，我们可以把需要设置的参数作为可配置的ROS参数，在yaml文件中传入程序。

## 自动查找方法

上述的调用方法都需要知道摄像头设备的索引（index）。在不同的设备上，同一设备的索引往往是不同的，取决于连接顺序。这导致我们的代码在部署时不够便利，需要将索引配置为一个可调整的参数。能否自动根据设备的唯一属性来查找，并读取画面呢？

答案是肯定的。考虑到这是一个实用的部署技巧，我们直接给出[C++实现](find_camera.cpp)。

在上述实现中，我们只需要给出相机设备的`"name"`属性和`"index"`属性（这和`/dev/videoX`的索引X不是一回事），就可以利用正则表达式找到该设备。
> **所有**属性都是以字符串形式给出的，请务必加上双引号。

这样的属性一般是唯一的、与设备关联的，我们可以用如下命令查找：
```bash
udevadm info -a -n /dev/videoX
```
当然，在查找属性的时候，你还是得先知道设备的索引。查找一次并确定后就再也不用了。这种方法在查找时，会给出子设备及其所属的所有设备（USB总线等）的各种属性，我选用了设备本身的`"name"`属性和`"index"`属性，你也可以使用更多的属性以唯一地确定设备（这得益于现代C++的`std::map`容器，类似字典，你可以加入更多键值对映射）。

## 虚拟设备方法

ROS为我们提供了一种信息流与硬件驱动解耦的数据传输方法，这使得我们在没有实际的物理硬件时，可以通过cv_bridge方法，将视频或图像加载为一个ROS话题，来模拟实际硬件。

那么利用OpenCV加载的方法就不可以吗？当然是可以的。`v4l2loopback`就是一个专门创建虚拟相机设备的功能包。

首先安装它：
```bash
sudo apt update
sudo apt install v4l2loopback-dkms v4l2loopback-utils
```
然后安装相关的python依赖：
```bash
pip install numpy opencv-python-headless pyfakewebcam
```
然后，需要将加载虚拟设备的命令授予无密码 sudo 权限，避免脚本调用它时权限不够。执行：
```bash
sudo visudo
```
在下面加入这一行：
```bash
zzy     ALL=(ALL) NOPASSWD: /usr/sbin/modprobe, /usr/sbin/rmmod
```
把开头的`zzy`换成你的用户名。然后你就可以运行[这个python脚本](image_test.py)。你可以在脚本的参数配置区定义：
```python
IMAGE_PATH = "/home/zzy/Airdrop_Drones/src/uav_perception/script/1.jpg"
VIRTUAL_CAM_NR = 20
FPS = 30
VIRTUAL_CAM_LABEL = "GazeboStereo"
```
它会自动地读取`IMAGE_PATH`路径下的图片，并将它拼接为一个*伪双目*图片（左右两边实际上是同一幅图），并以`FPS`指定的帧率，将画面持续发布在`/dev/video20`（即你设定的`VIRTUAL_CAM_NR`）上。程序会将虚拟设备的`"name"`属性指定为`"GazeboStereo"`（即你设定的`VIRTUAL_CAM_LABEL`），这样你就可以通过我们刚才的自动查找方法来查找相机设备，以保持仿真和实机代码的统一性。

这个脚本应该很容易被扩展到读取视频上，这里就不给出示例了。

为了与Gazebo仿真环境进行无缝集成，我们还有这样一种需求：将ROS话题发布的视频虚拟为设备并读取。
> 这有点“出口转内销再转出口”的感觉，但这是为了代码的统一性：希望仿真和实机环境部署的代码尽可能一致。

这个[python脚本](stereo_combiner.py)实现了订阅两个ROS话题，并将它们拼接为一个双目图像，然后发布到相机设备上的方法。原理和上面的脚本是一致的。

## 双目相机处理

前面我们提到了**双目图像**。对于USB双目相机（不是Intel Realsense系列），一般来说直接用OpenCV读取画面时，读取到的是原始的拼接画面，即左右目拼接在一起。
> 一般是通过设置分辨率来控制。例如说，设置为2560x720分辨率（如果相机支持），那么左右目就是1280x720分辨率图像；如果设置为1280x720，那么就只返回单目的图像。

> 顺便一提，其实realsense也不需要使用ROS就可以获得数据，因为它有自己的C++ API，realsense_ros只是对它的一种包装。原理和OpenCV的API是类似的。

为了分别使用双目图像，我们可以这样处理：
```cpp
/**
* @brief 将一个水平拼接的立体图像分割为左右两个视图。
*
* @param stitched_frame 输入的宽幅图像 (例如 1280x480)。
* @param left_image 输出的左视图 (引用)。
* @param right_image 输出的右视图 (引用)。
* @return 如果成功分割则返回 true，如果输入图像无效则返回 false。
*/

bool split_stereo_frame(const cv::Mat& stitched_frame, cv::Mat& left_image, cv::Mat& right_image) {
	if (stitched_frame.empty()) {
	std::cerr << "错误: 输入的拼接帧为空！" << std::endl;
	return false;
	}

	const int full_width = stitched_frame.cols;
	const int full_height = stitched_frame.rows;
	const int half_width = full_width / 2;

	if (full_width % 2 != 0) {
		std::cerr << "警告: 图像宽度为奇数，分割可能不精确。" << std::endl;
	}

	// 定义左半部分的矩形区域 (ROI)
	cv::Rect left_roi(0, 0, half_width, full_height);
	// 定义右半部分的矩形区域 (ROI)
	cv::Rect right_roi(half_width, 0, half_width, full_height);
 
	// 创建指向ROI的Mat对象 (无数据复制，非常高效)
	left_image = stitched_frame(left_roi);
	right_image = stitched_frame(right_roi);

	return true;
}
```
这种方法很高效：它不会创建图像的两份拷贝，而是给出了两个指向原图对应区域的引用。但是这种做法也有风险：它期望你不要轻易更改原图（例如转换通道等），否则引用会一并变更。
> 如果需要对左右目图像做同样的处理（例如将三通道变为单通道），请**不要分别对两个引用执行两次操作**，而是**在原始图像上操作一次**！否则可能会引起问题。

我个人的建议是，尽量避免不必要的拷贝。当然如果你觉得麻烦，使用`.clone()`方法就能规避上述问题了。

## 总结

这篇文章记录了与ROS解耦的图像加载方法。我们承认，ROS在开发和调试时是非常方便、实用的框架，便于我们将硬件与消息解耦，便捷地记录和回放数据。

然而，在工程部署中，与ROS解耦有时是更好的选择，避免了ROS节点的不稳定与额外开销。