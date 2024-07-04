#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/opencv.hpp>
#include <camera_info_manager/camera_info_manager.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace cam_pub_node{
    class CamNodelet : public nodelet::Nodelet {
    public:
        virtual void onInit() {
            ROS_INFO("Init CamNodelet...");

            // ROS 노드 핸들 초기화
            nh = getNodeHandle();
            nh_priv = getPrivateNodeHandle();

            // 파라미터 서버에서 파라미터 가져오기
            nh_priv.param<std::string>("topic_name", topic_name, "camera/image_raw");
            nh_priv.param<int>("camera_index", camera_index, 0);
            nh_priv.param<double>("fps", fps, 30.0);

            // 이미지 트랜스포트 및 퍼블리셔 초기화
            it = std::make_shared<image_transport::ImageTransport>(nh);
            pub = it->advertise(topic_name, 1);
            // info_pub = nh.advertise<sensor_msgs::CameraInfo>("camera/camera_info", 1);

            // // 카메라 정보 매니저 초기화
            // cinfo_manager.reset(new camera_info_manager::CameraInfoManager(nh, "camera"));

            ROS_INFO("camera index: %d", camera_index);
            ROS_INFO("topic name: %s", topic_name.c_str());
            ROS_INFO("fps: %f", fps);

            // 카메라 오픈
            cap.open(camera_index);
            if (!cap.isOpened()) {
                
                ROS_ERROR("Cannot open USB Camera");
                return;
            }
            cap.set(cv::CAP_PROP_FPS, fps);

            // 메인 루프
            timer = nh.createTimer(ros::Duration(1.0 / fps), &CamNodelet::timerCallback, this);
        }

    private:
        ros::NodeHandle nh;
        ros::NodeHandle nh_priv;
        image_transport::Publisher pub;
        ros::Publisher info_pub;
        std::shared_ptr<image_transport::ImageTransport> it;
        // std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_manager;
        cv::VideoCapture cap;
        std::string topic_name;
        int camera_index;
        double fps;
        ros::Timer timer;

        void timerCallback(const ros::TimerEvent& event) {
            cv::Mat frame, rgb_frame;
            sensor_msgs::ImagePtr msg;

            cap >> frame;
            if (!frame.empty()) {
                int channels = frame.channels();

                if (channels == 2) {
                    // YUYV를 BGR로 변환
                    cv::cvtColor(frame, rgb_frame, cv::COLOR_YUV2BGR_YUYV);
                } else if (channels == 3) {
                    rgb_frame = frame.clone();
                } else {
                    ROS_ERROR("지원되지 않는 채널 수입니다");
                    return;
                }

                msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rgb_frame).toImageMsg();
                pub.publish(msg);

                // 카메라 정보 퍼블리시
                // sensor_msgs::CameraInfo ci = cinfo_manager->getCameraInfo();
                // ci.header.stamp = msg->header.stamp;
                // info_pub.publish(ci);
            }
        }
    };
}
PLUGINLIB_EXPORT_CLASS(cam_pub_node::CamNodelet, nodelet::Nodelet)