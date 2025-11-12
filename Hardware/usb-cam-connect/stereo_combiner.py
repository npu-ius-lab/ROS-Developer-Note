#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import message_filters
import pyfakewebcam
import subprocess
import os
import sys
import time
import re

# ==============================================================================
# ---                           参数配置                                     ---
# ==============================================================================
# ROS话题
LEFT_IMAGE_TOPIC = "/camera/infra1/image_raw"
RIGHT_IMAGE_TOPIC = "/camera/infra2/image_raw"

# 虚拟摄像头参数 (这些参数必须与你的需求匹配)
VIRTUAL_CAM_WIDTH = 2560
VIRTUAL_CAM_HEIGHT = 720
VIRTUAL_CAM_NR = 10  # 将创建 /dev/video10
VIRTUAL_CAM_LABEL = "GazeboStereo"
VIRTUAL_CAM_PATH = f'/dev/video{VIRTUAL_CAM_NR}'

# ==============================================================================

def run_command(command, use_sudo=False):
    """
    辅助函数：执行一个shell命令并返回其输出和返回码。
    如果 use_sudo 为 True，则在命令前加上 'sudo'。
    """
    if use_sudo:
        command = f"sudo {command}"
    
    try:
        result = subprocess.run(
            command, shell=True, check=False, capture_output=True, text=True
        )
        return result.stdout.strip(), result.stderr.strip(), result.returncode
    except Exception as e:
        return "", str(e), -1

def setup_virtual_camera():
    """
    检查并配置 v4l2loopback 虚拟摄像头。
    此函数需要 root 权限才能成功执行 modprobe/rmmod。
    """
    rospy.loginfo("--- 正在检查并配置虚拟摄像头 ---")

    # 2. 检查设备是否存在且参数是否正确
    if os.path.exists(VIRTUAL_CAM_PATH):
        rospy.loginfo(f"设备 {VIRTUAL_CAM_PATH} 已存在，正在检查其参数...")
        stdout, _, retcode = run_command(f"v4l2-ctl -d {VIRTUAL_CAM_PATH} --info", use_sudo=True)
        if retcode == 0:
            # 使用正则表达式解析宽度和高度
            width_match = re.search(r'Width/Height\s*:\s*(\d+)\s*/\s*(\d+)', stdout)
            card_match = re.search(r'Card type\s*:\s*(.+)', stdout)
            
            if width_match and card_match:
                w, h = int(width_match.group(1)), int(width_match.group(2))
                label = card_match.group(1).strip()
                if w == VIRTUAL_CAM_WIDTH and h == VIRTUAL_CAM_HEIGHT and label == VIRTUAL_CAM_LABEL:
                    rospy.loginfo("虚拟摄像头参数正确，无需重新配置。")
                    return True
            
            rospy.logwarn("设备参数不正确或无法解析，将尝试重新加载模块。")
        else:
            rospy.logwarn(f"无法获取 {VIRTUAL_CAM_PATH} 的信息，将尝试重新加载模块。")

    # 3. 参数不正确或设备不存在，尝试重新加载模块
    rospy.loginfo("正在卸载旧的 v4l2loopback 模块 (如果存在)...")
    run_command("rmmod v4l2loopback", use_sudo=True)
    time.sleep(0.5) # 等待模块完全卸载

    rospy.loginfo("正在加载 v4l2loopback 模块并设置新参数...")
    modprobe_cmd = (
        f"modprobe v4l2loopback "
        f"video_nr={VIRTUAL_CAM_NR} "
        f"width={VIRTUAL_CAM_WIDTH} "
        f"height={VIRTUAL_CAM_HEIGHT} "
        f"card_label='{VIRTUAL_CAM_LABEL}' "
        f"exclusive_caps=1"
    )
    _, stderr, retcode = run_command(modprobe_cmd, use_sudo=True)
    
    if retcode != 0:
        rospy.logfatal(f"加载 v4l2loopback 模块失败: {stderr}")
        return False

    # 4. 最终验证
    time.sleep(0.5) # 等待 udev 创建设备文件
    if os.path.exists(VIRTUAL_CAM_PATH):
        rospy.loginfo("虚拟摄像头已成功创建！")
        return True
    else:
        rospy.logfatal(f"模块已加载，但设备 {VIRTUAL_CAM_PATH} 未能创建。")
        return False


class StereoCombiner:
    def __init__(self):
        rospy.init_node('stereo_to_v4l2_node', anonymous=True)
        
        # 在初始化任何东西之前，先配置好虚拟设备
        if not setup_virtual_camera():
            rospy.signal_shutdown("虚拟摄像头设置失败，节点将退出。")
            return

        self.bridge = CvBridge()
        
        try:
            self.fake_cam = pyfakewebcam.FakeWebcam(
                VIRTUAL_CAM_PATH, VIRTUAL_CAM_WIDTH, VIRTUAL_CAM_HEIGHT
            )
            rospy.loginfo(f"成功连接到虚拟摄像头 {VIRTUAL_CAM_PATH}")
        except Exception as e:
            rospy.logerr(f"无法打开虚拟摄像头 {VIRTUAL_CAM_PATH}: {e}")
            return

        left_sub = message_filters.Subscriber(LEFT_IMAGE_TOPIC, Image)
        right_sub = message_filters.Subscriber(RIGHT_IMAGE_TOPIC, Image)

        ts = message_filters.ApproximateTimeSynchronizer(
            [left_sub, right_sub], queue_size=10, slop=0.1
        )
        ts.registerCallback(self.image_callback)
        
        rospy.loginfo("等待同步的左右图像话题...")

    def image_callback(self, left_msg, right_msg):
        try:
            left_cv_image = self.bridge.imgmsg_to_cv2(left_msg, "bgr8")
            right_cv_image = self.bridge.imgmsg_to_cv2(right_msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        stitched_image = cv2.hconcat([left_cv_image, right_cv_image])

        h, w, _ = stitched_image.shape
        if w != VIRTUAL_CAM_WIDTH or h != VIRTUAL_CAM_HEIGHT:
            stitched_image = cv2.resize(stitched_image, (VIRTUAL_CAM_WIDTH, VIRTUAL_CAM_HEIGHT))

        frame_rgb = cv2.cvtColor(stitched_image, cv2.COLOR_BGR2RGB)
        self.fake_cam.schedule_frame(frame_rgb)

if __name__ == '__main__':
    try:
        combiner = StereoCombiner()
        # 只有在初始化成功后，combiner.fake_cam 才会存在
        if hasattr(combiner, 'fake_cam'):
            rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("节点关闭。")