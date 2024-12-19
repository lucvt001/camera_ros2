#ifndef PHYSICAL_CAMERA_HPP
#define PHYSICAL_CAMERA_HPP

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include "utils.h"
#include <camera_interfaces/srv/start_recording.hpp>
#include <camera_interfaces/srv/stop_recording.hpp>

class CameraPublisher : public rclcpp::Node
{
public:
    CameraPublisher();
    void initialize();
    void publishImage();

private:
    image_transport::Publisher image_pub_;
    cv::VideoCapture cap_;
    cv::Mat frame_, raw_frame_;
    int wb_temp_;
    sensor_msgs::msg::Image::SharedPtr msg_;
    std::string input_;
    std::string topic_name_;
    bool is_wsl2_;
    bool is_display_;
    bool is_recording_started_ = false;
    std::string mp4_output_folder_;
    cv::VideoWriter video_writer_;

    std::string calibration_yaml_path_;
    cv::Mat camera_matrix_, dist_coeffs_;
    cv::Mat map1_, map2_;
    bool is_calibration_enabled_ = false;
    void loadCameraCalibration();

    rclcpp::Service<camera_interfaces::srv::StartRecording>::SharedPtr start_recording_srv_;
    rclcpp::Service<camera_interfaces::srv::StopRecording>::SharedPtr stop_recording_srv_;
    void startRecordingCallback(const std::shared_ptr<camera_interfaces::srv::StartRecording::Request> request, std::shared_ptr<camera_interfaces::srv::StartRecording::Response> response);
    void stopRecordingCallback(const std::shared_ptr<camera_interfaces::srv::StopRecording::Request> request, std::shared_ptr<camera_interfaces::srv::StopRecording::Response> response);
};

cv::Mat resizeToFit(const cv::Mat &input_image, int max_width, int max_height);

#endif // PHYSICAL_CAMERA_HPP