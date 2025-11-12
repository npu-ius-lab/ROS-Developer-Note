# 一次性、集中书写、读取并加载全部参数的方法

> 作者：张子宇

在一些大型项目的开发中，需要配置很多参数。目前，我们的参数加载方式都遵循以下流程：
1. 将参数写在yaml或launch文件中；
2. 启动ROS节点时，将需要的参数加载到ROS服务器上；
3. 在程序中，为所需的参数设置一个变量；
4. 在程序中使用`nh.param`等ROS提供的方法，将ROS服务器的参数值加载到本地变量上。

可见，上述流程需要在**三个地方**书写同样含义的字段：
1. yaml文件或launch文件中，定义参数的值
2. 程序的变量定义中，定义参数的类型
3. 程序的初始化部分，使用`nh.param`将1中的值加载到2中

如果维护该项目需要频繁增删参数（例如说，新增一个PID控制器，则需要新增三个参数），那么上述流程无疑是很繁琐的。怎么样简化呢？
> 对于已有项目的参数设置，无需参照本文档，因为只需要改值，不用增减参数和变量。

本项目给出了一种方法，只需在**单个yaml文件中定义、书写参数**，就能直接在C++文件中使用。支持**运行时传入参数值**，支持**无限层次的字段嵌套**。

## 使用方法

考虑到ROS参数服务器修改参数非常便利，为了在运行时能够快速更改参数并查看效果（例如PID控制器），仍然将参数加载的流程与ROS集成。
> 换种方式说，我们将“运行时加载参数值”这个流程仍然交给ROS的`param`方法来做，我们的脚本只是根据yaml文件预编译出一个定义了所有参数字段的结构体。
> 
> 实际应用中，绝大多数参数都是静态的，只有少部分参数可能需要动态更新。为了利用ROS参数服务器的便捷机制，实现了一个update方法（从ROS服务器上实时拉取参数值），在需要使用动态参数的场合，只需要在循环时update参数的值即可。

使用方法非常简单：
1. 将你**唯一的参数文件**命名为config.yaml，并放在你ROS功能包中的config文件夹下；
2. 将[config.py](config.py)放在config文件夹下；
3. 在你功能包的CMakeLists.txt中，加入如下片段：
```cmake
# ==============================================================================
set(CONFIG_DIR ${CMAKE_CURRENT_SOURCE_DIR}/config)
set(CONFIG_YAML ${CONFIG_DIR}/config.yaml)
set(GENERATOR_SCRIPT ${CONFIG_DIR}/config.py)
set(GENERATED_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR}/include/${PROJECT_NAME})
set(GENERATED_CONFIG_H ${GENERATED_INCLUDE_DIR}/config.h)

add_custom_command(
	OUTPUT ${GENERATED_CONFIG_H}
	COMMAND ${Python3_EXECUTABLE} ${GENERATOR_SCRIPT}
	--output ${GENERATED_CONFIG_H}
	--name ${PROJECT_NAME}Config
	DEPENDS ${CONFIG_YAML} ${GENERATOR_SCRIPT}
	COMMENT "Generating C++ config header for package ${PROJECT_NAME}"
)

add_custom_target(
	${PROJECT_NAME}_config_generator
	DEPENDS ${GENERATED_CONFIG_H}
)

include_directories(
	${CMAKE_CURRENT_BINARY_DIR}/include
)

# ==============================================================================
```
同时在你的主程序的`add_executable`后面加入这样一行：
```cmake
add_dependencies(your_node ${PROJECT_NAME}_config_generator)
```
记得将`your_node`改为你的`add_executable`中的可执行文件的名称。

4. 编译一次程序。然后，在需要调用参数的代码中，这样使用参数：
```cpp
#include <${PROJECT_NAME}/config.h> // ${PROJECT_NAME}是你CMake中的项目名称

{PROJECT_NAME}_config::{PROJECT_NAME}Config params_;
double kp = params_.pid.kp;
```
事实上，Python脚本为你的项目自动生成了一个结构体定义，这个结构体的`namespace`为`{PROJECT_NAME}_config`，结构体类型名称为`{PROJECT_NAME}Config`。
> 例如，你的项目名为`mission`，那么定义此结构体的方法即为：`mission_config::missionConfig params_`。

5. 启动节点时，通过launch文件传递参数给你的节点。例如说：
```xml
<launch>
	<node name="your_node" pkg="your_pkg" type="your_node" output="screen">
		<rosparam file="$(find your_node)/config/config.yaml" command="load" />
	</node>
</launch>
```
注意将`your_node`和`your_pkg`改为你的节点和功能包的名称。

> 请注意，这种方式会将参数传递在节点的私有命名空间内。这是必须的，因此若要利用ROS服务器动态修改参数，请注意不能将参数加载到全局命名空间。

6. 当你想添加参数时，只需要编辑yaml文件，然后执行一次编译（或者VSCode的“生成”按钮），新的参数定义就会自动加入结构体。
7. 关于**同类型不同名的参数结构体加载**：
	如果我们想要加载定义相同，但名称不同的若干参数块（例如，若干个PID控制器）：
	```yaml
	PID_x: 
		$type: PIDConfig
		kp: 1.0
		ki: 0.1
		kd: 0.01
	PID_y:
		$type: PIDConfig
		kp: 2.0
		ki: 0.2
		kd: 0.02
	```
	如果不显式声明`$type`，那么脚本将会为`PID_x`和`PID_y`生成两个独立的结构体定义`PID_xConfig`和`PID_yConfig`，但这实际上是冗余定义，它们的**成员结构和名称**是完全一致的。
	
	为此，我们在定义参数块时，定义`$type: PIDConfig`，那么脚本就会自动为它们生成一个结构体定义PIDConfig，并生成两个该类型的不同成员。注意，如果使用了这个标签，那么要保证所有使用该标签的参数块内的字段名称和类型相同，否则**会出现难以察觉的静默错误**。
## 原理说明

对于C++编程来说，实现上述功能是比较困难的。因为C++缺乏原生的**反射**机制。

所谓反射，指的是程序能够在**运行时**获得变量信息的一种能力。对于C++这样的**编译型语言**来说，所有在代码中书写的变量信息，在编译时就被丢弃了：编译器会为指定类型的变量分配指定大小的空间，然后变量会变为指向特定地址的偏移量，在程序**运行时**，我们再也无从获知变量的名称。自然，也无法获得它的具体成员数量、成员名称、成员值等。

这种机制的缺失会导致：我们无法在程序编译以后，再确定程序中的参数是什么类型，有什么值。本质上来说，我们想要实现的目标就是：**仅在yaml文件中书写一次参数和它的值，就能够实现参数变量定义、参数值加载的工作**。

为了避免缺乏反射机制带来的问题，我们采用“曲线救国”的方式：用一个脚本读取yaml文件中的所有参数，根据参数的值自动地生成结构体和变量名，并将其填充到一个.h文件中。这样，在编译器编译C++代码时，就能自动地找到变量定义了。至于参数加载，我们也可以将`nh.param`方式写在结构体的初始化方法中。
