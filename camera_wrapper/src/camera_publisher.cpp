#include <camera_wrapper/camera_publisher.hpp>

CameraPublisher::CameraPublisher() : Node("camera_publisher")
{
    // Placeholder
}

void CameraPublisher::initialize() 
{
    // Get the topic name from the parameter server
    this->declare_parameter<std::string>("input", "/dev/video0");
    this->declare_parameter<std::string>("topic_name", "/sensor/camera");
    this->declare_parameter<bool>("is_wsl2", false);
    this->declare_parameter<bool>("is_display", false);
    this->declare_parameter<std::string>("mp4_output_folder", "");
    this->declare_parameter<std::string>("calibration_yaml_path", "");
    this->declare_parameter<int>("whitebalance_temperature", -1);
    
    this->get_parameter("input", input_);
    this->get_parameter("topic_name", topic_name_);
    this->get_parameter("is_wsl2", is_wsl2_);
    this->get_parameter("is_display", is_display_);
    this->get_parameter("mp4_output_folder", mp4_output_folder_);
    this->get_parameter("calibration_yaml_path", calibration_yaml_path_);
    this->get_parameter("whitebalance_temperature", wb_temp_);

    // Initialize the service servers
    start_recording_srv_ = this->create_service<camera_interfaces::srv::StartRecording>(
        "start_recording", std::bind(&CameraPublisher::startRecordingCallback, this, std::placeholders::_1, std::placeholders::_2));
    stop_recording_srv_ = this->create_service<camera_interfaces::srv::StopRecording>(
        "stop_recording", std::bind(&CameraPublisher::stopRecordingCallback, this, std::placeholders::_1, std::placeholders::_2));

    // Open the camera in WSL2
    if (is_wsl2_) 
    {
        cap_.open(input_, cv::CAP_V4L2);
        cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
        RCLCPP_WARN(this->get_logger(), "WSL2 mode. Are you sure?");
    }
    else 
        cap_.open(input_);

    if (!cap_.isOpened()) 
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open camera: %s", input_.c_str());
        rclcpp::shutdown();
    }
    else 
        RCLCPP_INFO(this->get_logger(), "Camera opened: %s", input_.c_str());

    // Load the camera calibration parameters
    if (calibration_yaml_path_ != "") 
        loadCameraCalibration();
    else 
        RCLCPP_INFO(this->get_logger(), "Camera calibration file NOT provided!");

    // Whitebalance the camera
    if (wb_temp_ != -1) 
    {
        cap_.set(cv::CAP_PROP_AUTO_WB, 0);
        cap_.set(cv::CAP_PROP_WB_TEMPERATURE, wb_temp_);
        RCLCPP_INFO(this->get_logger(), "Whitebalance set to: %d", wb_temp_);
    }
    else cap_.set(cv::CAP_PROP_AUTO_WB, 1);

    if (mp4_output_folder_ == "")
        RCLCPP_WARN(this->get_logger(), "MP4 recording is NOT enabled!");
    else if (mp4_output_folder_.back() == '/')
        mp4_output_folder_.pop_back();

    image_transport::ImageTransport it(shared_from_this());
    image_pub_ = it.advertise(topic_name_, 1);
    RCLCPP_INFO(this->get_logger(), "Publishing camera images to topic: %s", topic_name_.c_str());
}

void CameraPublisher::publishImage() 
{
    if (is_calibration_enabled_) {
        cap_ >> raw_frame_; // Capture a frame
        cv::remap(raw_frame_, frame_, map1_, map2_, cv::INTER_LINEAR); // Undistort the frame
    }
    else cap_ >> frame_; // Capture a frame

    if (is_display_) {
        cv::imshow("frame", frame_); // Display the frame
        cv::waitKey(10);
    }

    if (!frame_.empty()) 
    {
        // Convert the frame to a ROS image message
        msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame_).toImageMsg();
        image_pub_.publish(msg_);
    }

    if (video_writer_.isOpened()) 
        video_writer_ << frame_; // Write the frame to the video
}

void CameraPublisher::startRecordingCallback(const std::shared_ptr<camera_interfaces::srv::StartRecording::Request> request, std::shared_ptr<camera_interfaces::srv::StartRecording::Response> response) {
    if (is_recording_started_) {
        RCLCPP_WARN(this->get_logger(), "Recording already started!");
        response->is_success = false;
        return;
    } else if (mp4_output_folder_ == "") {
        RCLCPP_ERROR(this->get_logger(), "MP4 recording is NOT enabled!");
        response->is_success = false;
        return;
    }

    std::string mp4_file_name = "";
    mp4_file_name = generateMP4FileName();
    mp4_file_name = mp4_output_folder_ + "/" + mp4_file_name;
    RCLCPP_WARN(this->get_logger(), "Saving MP4 video to: %s", mp4_file_name.c_str());

    int frame_width = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH));
    int frame_height = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT));
    video_writer_.open(mp4_file_name, cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 30, cv::Size(frame_width, frame_height), true);

    if (!video_writer_.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open video writer");
        response->is_success = false;
    } else {
        response->is_success = true;
        is_recording_started_ = true;
    }
}

void CameraPublisher::stopRecordingCallback(const std::shared_ptr<camera_interfaces::srv::StopRecording::Request> request, std::shared_ptr<camera_interfaces::srv::StopRecording::Response> response) {
    if (!is_recording_started_) {
        RCLCPP_WARN(this->get_logger(), "Recording is NOT started!");
        response->is_success = false;
        return;
    }
    video_writer_.release();
    is_recording_started_ = false;
    RCLCPP_WARN(this->get_logger(), "Recording stopped!");
    response->is_success = true;
}

void CameraPublisher::loadCameraCalibration() 
{
    // Load the camera calibration parameters from the YAML file
    cv::FileStorage fs(calibration_yaml_path_, cv::FileStorage::READ);
    if (!fs.isOpened()) 
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open camera calibration file: %s", calibration_yaml_path_.c_str());
        rclcpp::shutdown();
    }

    fs["Camera_Matrix"] >> camera_matrix_;
    fs["Distortion_Coefficients"] >> dist_coeffs_;
    fs.release();

    // Check the dimensions of the loaded matrices
    if (camera_matrix_.rows != 3 || camera_matrix_.cols != 3) {
        RCLCPP_ERROR(this->get_logger(), "Invalid camera matrix dimensions!");
    }

    if (dist_coeffs_.rows != 5 || dist_coeffs_.cols != 1) {
        RCLCPP_ERROR(this->get_logger(), "Invalid distortion coefficients dimensions!");
    }

    cv::Size imageSize = cv::Size(static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH)), static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT)));

    cv::initUndistortRectifyMap(
                camera_matrix_, dist_coeffs_, cv::Mat(),
                cv::getOptimalNewCameraMatrix(camera_matrix_, dist_coeffs_, imageSize, 0, imageSize, 0), imageSize,
                CV_16SC2, map1_, map2_);

    RCLCPP_WARN(this->get_logger(), "Camera calibration enabled!");
    is_calibration_enabled_ = true;
}