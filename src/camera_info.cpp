#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "sensor_msgs/msg/image.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class CameraPublisher : public rclcpp::Node
{


  public:
    CameraPublisher()
    : Node("camera_info")
    {
      publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera_info_publisher", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&CameraPublisher::timer_callback, this));
      std::string url = "http://192.168.1.150:8080/stream/video/mjpeg?    resolution=HD&&Username=admin&&Password=YXJnb3RlYW0=&&tempid=0.04348    721299552394";
      cap.open(url);
      if(!cap.isOpened()){
	RCLCPP_ERROR(this->get_logger(), "Could not open video stream");
        throw std::runtime_error("Could not open video stream");
    }
      freq_ = this->declare_parameter("frequency", 30.0);
      RCLCPP_WARN(this->get_logger(), "ababa");   

      timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / freq_)), 
        std::bind(&CameraPublisher::timer_callback, this));
	
    }
  private:
    void timer_callback()
    {
	cv::Mat frame;
	cap >> frame;
	RCLCPP_WARN(this->get_logger(), "dupa");
	if(frame.empty())
	{
	RCLCPP_WARN(this->get_logger(), "Empty frame captured");
            return;
	}
	cv_bridge::CvImage img_bridge;
       sensor_msgs::msg::Image img_msg;
       std_msgs::msg::Header header; // empty header
       header.stamp = this->get_clock()->now();
       header.frame_id = frame_id_;

	img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, frame);
      	img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::msg::Image
      	publisher_->publish(img_msg);
    }


	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
	double freq_;
	cv::VideoCapture cap;
	std::string frame_id_;

    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraPublisher>());
  rclcpp::shutdown();
  return 0;
}
