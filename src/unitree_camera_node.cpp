#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <UnitreeCameraSDK.hpp>
#include <opencv2/opencv.hpp>

int main(int argc, char **argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "unitree_camera_node");
    ros::NodeHandle nh("~");
    ros::NodeHandle nh_pub;

    // 获取参数（设备节点、帧大小、帧率）
    int deviceNode;
    nh.param("device_node", deviceNode, 0); // 默认设备节点为0 (/dev/video0)
    int width, height, fps;
    nh.param("frame_width", width, 1856);   // 默认宽度1856
    nh.param("frame_height", height, 800);  // 默认高度800
    nh.param("fps", fps, 30);               // 默认帧率30
    cv::Size frameSize(width, height);

    // 获取配置文件路径参数
    std::string config_file;
    if (!nh.getParam("config_file", config_file)) {
        ROS_ERROR("Failed to get 'config_file' parameter");
        return -1; // 如果参数未设置，退出程序
    }

    // 使用从参数服务器读取的路径初始化 UnitreeCamera
    UnitreeCamera cam(config_file);
    if (!cam.isOpened()) {
        ROS_ERROR("Failed to open camera with config: %s", config_file.c_str());
        return -1; // 如果相机打开失败，退出程序
    }
    
    // 设置相机参数
    cam.setRawFrameSize(frameSize); // 设置原始帧大小
    cam.setRawFrameRate(fps);       // 设置帧率
    cam.setRectFrameSize(cv::Size(frameSize.width >> 1, frameSize.height >> 0)); // 设置矫正帧大小
    cam.startCapture(); // 开始捕获

    // 创建图像发布者
    image_transport::ImageTransport it(nh_pub);
    image_transport::Publisher pub_left = it.advertise("/camera/infra1/image_rect_raw", 1);
    image_transport::Publisher pub_right = it.advertise("/camera/infra2/image_rect_raw", 1);

    // 设置循环频率
    ros::Rate loop_rate(fps);
    while (ros::ok()) {
        cv::Mat left, right;
        if (cam.getRectStereoFrame(left, right)) {
            cv::flip(left, left, -1);   // 翻转图像
            cv::flip(right, right, -1); // 翻转图像
            // 将cv::Mat转换为sensor_msgs::Image消息
            sensor_msgs::ImagePtr msg_left = cv_bridge::CvImage(std_msgs::Header(), "bgr8", left).toImageMsg();
            sensor_msgs::ImagePtr msg_right = cv_bridge::CvImage(std_msgs::Header(), "bgr8", right).toImageMsg();

            // 设置时间戳
            auto time = ros::Time::now();
            msg_left->header.stamp = time;
            msg_right->header.stamp = time;

            // 发布左右目图像
            pub_left.publish(msg_right);
            pub_right.publish(msg_left);
        } else {
            ROS_WARN("Failed to get rectified stereo frame");
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    // 停止捕获
    cam.stopCapture();
    return 0;
}
