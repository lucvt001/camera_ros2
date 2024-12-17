#include <camera_wrapper/camera_publisher.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraPublisher>();
    node->initialize();

    int frame_rate;
    node->declare_parameter<int>("frame_rate", 10);
    node->get_parameter("frame_rate", frame_rate);
    std::cout << "Frame rate: " << frame_rate << std::endl;

    rclcpp::Rate loop_rate(frame_rate);

    while (rclcpp::ok()) 
    {
        node->publishImage();
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
