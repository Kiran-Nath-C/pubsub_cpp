#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

class ImageSubscriber : public rclcpp::Node {
public:
  ImageSubscriber() : Node("image_subscriber") {
    image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "image_topic", 10, std::bind(&ImageSubscriber::imageCallback, this, std::placeholders::_1));

    filtered_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("filtered_image_topic", 10);
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
      // Convert ROS Image message to OpenCV image
      cv::Mat cv_image = cv_bridge::toCvCopy(msg, "bgr8")->image;

      // Apply filter (grayscale conversion)
      cv::cvtColor(cv_image, cv_image, cv::COLOR_BGR2HSV);

      // Convert OpenCV image to ROS Image message
      auto filtered_image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", cv_image).toImageMsg();

      // Convert the shared_ptr to a unique_ptr
      auto unique_filtered_image_msg = std::make_unique<sensor_msgs::msg::Image>(*filtered_image_msg);

      // Publish the filtered image using unique_ptr
      filtered_image_publisher_->publish(std::move(unique_filtered_image_msg));

    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "CV_Bridge exception: %s", e.what());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr filtered_image_publisher_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageSubscriber>());
  rclcpp::shutdown();
  return 0;
}

