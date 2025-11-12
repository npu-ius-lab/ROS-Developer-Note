#include <ros/ros.h>
#include <opencv2/opencv.hpp>

// 将所有逻辑封装在一个类中
class DirectCameraNode
{
public:
    // 构造函数
    DirectCameraNode() : nh_("~"), it_(nh_) {
        camera_index_ = 0;
        frame_rate_ = 30;

        // 打开摄像头
        cap_.open(camera_index_);
        if (!cap_.isOpened()) {
            ROS_ERROR_STREAM("Failed to open camera with index " << camera_index_);
            // 让节点崩溃，因为没有摄像头就无法工作
            ros::shutdown();
            return;
        }
        ROS_INFO_STREAM("Successfully opened camera with index " << camera_index_);
	cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
    	cap_.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
    	cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
    	cap_.set(cv::CAP_PROP_FPS, 30);
    	
        // 创建定时器
        timer_ = nh_.createTimer(ros::Duration(1.0 / frame_rate_), &DirectCameraNode::timerCallback, this);
    }

    // 析构函数，用于释放资源
    ~DirectCameraNode() {
        if (cap_.isOpened()) {
            cap_.release();
            ROS_INFO("Camera released.");
        }
    }

private:
    // 私有变量
    ros::NodeHandle nh_;
    ros::Timer timer_;

    cv::VideoCapture cap_;
    int camera_index_;
    int frame_rate_;

    // 定时器回调函数，每次触发时执行
    void timerCallback(const ros::TimerEvent& event) {
        cv::Mat frame;
        // 尝试从摄像头抓取一个新帧到内部缓存
        if (cap_.grab()) {
            // 只有在成功抓取到新帧时，才进行解码和处理
            cv::Mat frame;
            cap_.retrieve(frame);

            if (frame.empty()) {
                ROS_WARN("Grabbed a frame, but it's empty.");
                return;
            }
        }
        // 可以传递frame变量以完成之后的图像处理
    }

};


int main(int argc, char** argv) {
    ros::init(argc, argv, "direct_camera_node");
    
    DirectCameraNode node;
    ros::spin();

    return 0;
}
