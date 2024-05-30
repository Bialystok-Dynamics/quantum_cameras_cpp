#include <string>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/msg/image.hpp>

using namespace std::chrono_literals;

class CameraPublisher : public rclcpp::Node
{
public:
    CameraPublisher(const std::string& url, int width, int height)
    : Node("camera_info"), url_(url), width_(width), height_(height)
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera_info_publisher", 10);
        freq_ = this->declare_parameter("frequency", 30.0);

        cap.open(url_);
        if (!cap.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Could not open video stream");
            throw std::runtime_error("Could not open video stream");
        }

        if (width_ > 0 && height_ > 0) {
            cap.set(cv::CAP_PROP_FRAME_WIDTH, width_);
            cap.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
        }

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / freq_)),
            std::bind(&CameraPublisher::timer_callback, this)
        );
    }

private:
    void timer_callback()
    {
        cv::Mat frame;
        cap >> frame;

        if (frame.empty()) {
            RCLCPP_WARN(this->get_logger(), "Empty frame captured");
            return;
        }

        cv_bridge::CvImage img_bridge;
        sensor_msgs::msg::Image img_msg;
        std_msgs::msg::Header header;
        header.stamp = this->get_clock()->now();
        header.frame_id = "camera_frame";

        img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, frame);
        img_bridge.toImageMsg(img_msg);
        publisher_->publish(img_msg);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    double freq_;
    cv::VideoCapture cap;
    std::string url_;
    int width_;
    int height_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    if (argc < 2) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Usage: camera_publisher <camera_url> [width height]");
        return 1;
    }

    std::string url = argv[1];
    int width = argc > 2 ? std::stoi(argv[2]) : 0;
    int height = argc > 3 ? std::stoi(argv[3]) : 0;

    auto node = std::make_shared<CameraPublisher>(url, width, height);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

