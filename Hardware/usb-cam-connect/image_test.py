#!/usr/bin/env python3

import cv2
import numpy as np
import pyfakewebcam
import time
import os
import sys
import subprocess
import re


IMAGE_PATH = "/home/zzy/Airdrop_Drones/src/uav_perception/script/1.jpg"
VIRTUAL_CAM_NR = 20
FPS = 30
VIRTUAL_CAM_LABEL = "GazeboStereo"

# ==============================================================================
# ---                  辅助函数 (用于日志和命令执行)                          ---
# ==============================================================================
def log_info(msg):
    print(f"[INFO] {msg}")

def log_warn(msg):
    print(f"[WARN] {msg}", file=sys.stderr)

def log_fatal(msg):
    print(f"[FATAL] {msg}", file=sys.stderr)

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

# ==============================================================================
# ---                  虚拟摄像头设置函数                                  ---
# ==============================================================================
def setup_virtual_camera(cam_nr, width, height, label):
    """
    检查并配置 v4l2loopback 虚拟摄像头。
    此函数需要 root 权限才能成功执行 modprobe/rmmod。
    """
    log_info("--- 正在检查并配置虚拟摄像头 ---")
    cam_path = f'/dev/video{cam_nr}'

    #if os.geteuid() != 0:
    ####    log_fatal("脚本需要 root 权限来加载内核模块。")
    ###   log_fatal(f"请使用 'sudo python3 {sys.argv[0]} ...' 运行。")
    #    return False

    if os.path.exists(cam_path):
        log_info(f"设备 {cam_path} 已存在，正在检查其参数...")
        stdout, _, retcode = run_command(f"v4l2-ctl -d {cam_path} --info", use_sudo=True)
        if retcode == 0:
            width_match = re.search(r'Width/Height\s*:\s*(\d+)\s*/\s*(\d+)', stdout)
            card_match = re.search(r'Card type\s*:\s*(.+)', stdout)
            
            if width_match and card_match:
                w, h = int(width_match.group(1)), int(width_match.group(2))
                l = card_match.group(1).strip()
                if w == width and h == height and l == label:
                    log_info("虚拟摄像头参数正确，无需重新配置。")
                    return True
            log_warn("设备参数不正确或无法解析，将尝试重新加载模块。")
        else:
            log_warn(f"无法获取 {cam_path} 的信息，将尝试重新加载模块。")

    log_info("正在卸载旧的 v4l2loopback 模块 (如果存在)...")
    run_command("rmmod v4l2loopback", use_sudo=True) 
    time.sleep(0.5)

    log_info("正在加载 v4l2loopback 模块并设置新参数...")
    modprobe_cmd = (
        f"modprobe v4l2loopback "
        f"video_nr={cam_nr} "
        f"width={width} "
        f"height={height} "
        f"card_label='{label}' "
        f"exclusive_caps=1"
    )
    _, stderr, retcode = run_command(modprobe_cmd, use_sudo=True)
    
    if retcode != 0:
        log_fatal(f"加载 v4l2loopback 模块失败: {stderr}")
        return False

    time.sleep(0.5)
    if os.path.exists(cam_path):
        log_info("虚拟摄像头已成功创建！")
        return True
    else:
        log_fatal(f"模块已加载，但设备 {cam_path} 未能创建。")
        return False

# ==============================================================================
# ---                           主函数                                       ---
# ==============================================================================
def main():
    # 1. 加载并处理图片
    log_info(f"正在加载图片: {IMAGE_PATH}")
    image = cv2.imread(IMAGE_PATH)
    if image is None:
        log_fatal("无法加载图片，请检查路径是否正确。")
        sys.exit(1)

    # 2. 创建伪双目拼接图
    stitched_image = cv2.hconcat([image, image])
    height, width, _ = stitched_image.shape
    log_info(f"创建拼接图，分辨率: {width}x{height}")

    # 3. 设置虚拟摄像头 (使用拼接图的尺寸)
    cam_label = VIRTUAL_CAM_LABEL
    if not setup_virtual_camera(VIRTUAL_CAM_NR, width, height, cam_label):
        sys.exit(1)

    # 4. 初始化虚拟摄像头推流器
    cam_path = f'/dev/video{VIRTUAL_CAM_NR}'
    try:
        fake_cam = pyfakewebcam.FakeWebcam(cam_path, width, height)
    except Exception as e:
        log_fatal(f"无法连接到 {cam_path}: {e}")
        sys.exit(1)

    # 5. 开始循环推流
    log_info(f"开始以 {FPS} FPS 推流到 {cam_path}...")
    log_info("按 Ctrl+C 停止。")
    
    # 转换颜色空间一次即可，因为图片是静态的
    frame_rgb = cv2.cvtColor(stitched_image, cv2.COLOR_BGR2RGB)
    
    try:
        while True:
            fake_cam.schedule_frame(frame_rgb)
            time.sleep(1.0 / FPS)
    except KeyboardInterrupt:
        log_info("\n检测到 Ctrl+C，正在关闭...")
    finally:
        # 最好也把模块卸载掉，保持系统干净
        log_info("正在卸载 v4l2loopback 模块...")
        run_command("rmmod v4l2loopback")
        log_info("脚本已退出。")


if __name__ == "__main__":
    main()