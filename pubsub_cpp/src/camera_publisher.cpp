#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

class CameraPublisher : public rclcpp::Node {
public:
  CameraPublisher() : Node("camera_publisher") {
    image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image_topic", 10);
    cap_.open(0);  // Adjust the parameter if your camera is not the default (e.g., 1 for the second camera)

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&CameraPublisher::publishImage, this));
  }

private:
  void publishImage() {
    if (!cap_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Error: Couldn't open camera");
      return;
    }

    cv::Mat frame;
    cap_ >> frame;

    if (frame.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Error: Couldn't capture frame");
      return;
    }

    auto ros_image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();

    // Convert the shared_ptr to a unique_ptr
    auto unique_ros_image_msg = std::make_unique<sensor_msgs::msg::Image>(*ros_image_msg);

    // Publish the image using unique_ptr
    image_publisher_->publish(std::move(unique_ros_image_msg));

    // Display the image (optional)
    
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  cv::VideoCapture cap_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraPublisher>());
  rclcpp::shutdown();
  return 0;
}

