import cv2
import time

def show_camera_feed():
    """
    打开默认摄像头并实时显示画面。
    按 'q' 键退出。
    """
    # 摄像头设备索引，0 通常是默认摄像头
    camera_index = 2
    cap = cv2.VideoCapture(camera_index)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_AUTO_WB, 1)


    if not cap.isOpened():
        print(f"错误：无法打开索引为 {camera_index} 的摄像头。")
        return

    print("摄像头已启动。按 'q' 键退出。")

    window_name = 'Resizable Camera Feed'
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL) 

    # 用于计算FPS的变量
    # time.time() 返回当前时间的时间戳
    start_time = time.time()
    frame_counter = 0
    fps_text = "FPS: N/A"

    while True:
        ret, frame = cap.read()
        if not ret:
            print("错误：无法读取视频帧。")
            break

        # 计算FPS
        frame_counter += 1
        # 计算从start_time到现在的总时间
        elapsed_time = time.time() - start_time

        # 每隔1秒钟更新一次FPS数值
        if elapsed_time > 1.0:
            fps = frame_counter / elapsed_time
            fps_text = f"FPS: {fps:.2f}" # 格式化为两位小数
            
            # 重置计数器和开始时间
            frame_counter = 0
            start_time = time.time()
        
        # 将FPS文本绘制到画面上
        # 参数: 图像, 要绘制的文本, 文本左下角坐标, 字体, 字体大小, 颜色(BGR), 线条粗细
        cv2.putText(frame, fps_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # 显示最终的画面
        cv2.imshow(window_name, frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    print("程序已关闭。")

if __name__ == "__main__":
    show_camera_feed()