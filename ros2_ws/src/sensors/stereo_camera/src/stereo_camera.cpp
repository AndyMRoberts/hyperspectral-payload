#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>

#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace
{

std::string make_gstreamer_pipeline(int sensor_id, int width, int height, int fps)
{
  std::ostringstream pipeline;
  pipeline
    << "nvarguscamerasrc sensor-id=" << sensor_id
    << " ! video/x-raw(memory:NVMM), width=" << width
    << ", height=" << height
    << ", format=(string)NV12, framerate=(fraction)" << fps << "/1"
    << " ! nvvidconv flip-method=2"
    << " ! video/x-raw, width=" << width
    << ", height=" << height
    << ", format=(string)BGRx"
    << " ! videoconvert"
    << " ! video/x-raw, format=(string)BGR"
    << " ! appsink";
  return pipeline.str();
}

void mat_to_image_msg(
  const cv::Mat & mat, const std::string & encoding,
  const std_msgs::msg::Header & header, sensor_msgs::msg::Image & msg)
{
  if (mat.empty()) {
    throw std::runtime_error("Cannot convert empty cv::Mat to sensor_msgs/Image");
  }

  msg.header = header;
  msg.height = static_cast<uint32_t>(mat.rows);
  msg.width = static_cast<uint32_t>(mat.cols);
  msg.encoding = encoding;
  msg.is_bigendian = false;
  msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(mat.cols * mat.elemSize());
  msg.data.resize(static_cast<size_t>(mat.rows) * msg.step);

  if (mat.isContinuous()) {
    std::memcpy(msg.data.data(), mat.data, msg.data.size());
  } else {
    for (int row = 0; row < mat.rows; ++row) {
      const auto row_bytes = static_cast<size_t>(msg.step);
      std::memcpy(
        msg.data.data() + row * row_bytes,
        mat.ptr(row),
        row_bytes);
    }
  }
}

}  // namespace

class StereoCameraNode : public rclcpp::Node
{
public:
  StereoCameraNode()
  : Node("stereo_camera")
  {
    acquisition_rate_hz_ = declare_parameter<double>("acquisition_rate_hz", 20.0);
    throttled_rate_hz_ = declare_parameter<double>("throttled_rate_hz", 1.0);
    camera_width_ = declare_parameter<int>("camera_width", 640);
    camera_height_ = declare_parameter<int>("camera_height", 480);
    frame_id_ = declare_parameter<std::string>("frame_id", "stereo_camera");
    left_topic_ = declare_parameter<std::string>("left_topic", "/sensors/stereo/left");
    right_topic_ = declare_parameter<std::string>("right_topic", "/sensors/stereo/right");

    if (acquisition_rate_hz_ <= 0.0) {
      throw std::runtime_error("acquisition_rate_hz must be > 0");
    }
    if (throttled_rate_hz_ <= 0.0) {
      throw std::runtime_error("throttled_rate_hz must be > 0");
    }
    if (camera_width_ <= 0 || camera_height_ <= 0) {
      throw std::runtime_error("camera_width and camera_height must be > 0");
    }

    const int capture_fps = static_cast<int>(std::lround(acquisition_rate_hz_));
    open_cameras(capture_fps);

    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.best_effort();
    left_pub_ = create_publisher<sensor_msgs::msg::Image>(left_topic_, qos);
    right_pub_ = create_publisher<sensor_msgs::msg::Image>(right_topic_, qos);
    left_throttled_pub_ = create_publisher<sensor_msgs::msg::Image>(left_topic_ + "/throttled", qos);
    right_throttled_pub_ = create_publisher<sensor_msgs::msg::Image>(right_topic_ + "/throttled", qos);

    const auto acquisition_period = std::chrono::duration<double>(1.0 / acquisition_rate_hz_);
    acquisition_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(acquisition_period),
      std::bind(&StereoCameraNode::acquisition_callback, this));

    const auto throttled_period = std::chrono::duration<double>(1.0 / throttled_rate_hz_);
    throttled_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(throttled_period),
      std::bind(&StereoCameraNode::throttled_callback, this));

    RCLCPP_INFO(
      get_logger(),
      "Stereo camera node started: acquisition_rate_hz=%.3f throttled_rate_hz=%.3f size=%dx%d "
      "(cam0=right, cam1=left)",
      acquisition_rate_hz_, throttled_rate_hz_, camera_width_, camera_height_);
    RCLCPP_INFO(
      get_logger(),
      "Publishing raw left='%s' right='%s' throttled left='%s/throttled' right='%s/throttled'",
      left_topic_.c_str(), right_topic_.c_str(), left_topic_.c_str(), right_topic_.c_str());
  }

private:
  void open_cameras(int capture_fps)
  {
    const std::string cam0_pipeline = make_gstreamer_pipeline(
      0, camera_width_, camera_height_, capture_fps);
    const std::string cam1_pipeline = make_gstreamer_pipeline(
      1, camera_width_, camera_height_, capture_fps);

    cam0_.open(cam0_pipeline, cv::CAP_GSTREAMER);
    if (!cam0_.isOpened()) {
      throw std::runtime_error("cam0 is not opened (sensor-id=0)");
    }

    cam1_.open(cam1_pipeline, cv::CAP_GSTREAMER);
    if (!cam1_.isOpened()) {
      throw std::runtime_error("cam1 is not opened (sensor-id=1)");
    }

    RCLCPP_INFO(get_logger(), "Opened cam0 (right) and cam1 (left)");
  }

  void acquisition_callback()
  {
    cv::Mat frame_right;
    cv::Mat frame_left;
    // cam0_ >> frame_left;
    // cam1_ >> frame_right;
    cam0_.grab();
    cam1_.grab();
    // this allows better syncing by grabbing first, then decoding after
    cam0_.retrieve(frame_left);
    cam1_.retrieve(frame_right);

    if (!frame_left.data || !frame_right.data) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Error reading images from cam0/cam1");
      return;
    }

    std_msgs::msg::Header header;
    header.stamp = now();
    header.frame_id = frame_id_;

    sensor_msgs::msg::Image left_msg;
    sensor_msgs::msg::Image right_msg;
    mat_to_image_msg(frame_left, "bgr8", header, left_msg);
    mat_to_image_msg(frame_right, "bgr8", header, right_msg);

    left_pub_->publish(left_msg);
    right_pub_->publish(right_msg);

    {
      std::lock_guard<std::mutex> lock(latest_frames_mutex_);
      latest_left_msg_ = left_msg;
      latest_right_msg_ = right_msg;
      have_latest_frames_ = true;
    }
  }

  void throttled_callback()
  {
    sensor_msgs::msg::Image left_msg;
    sensor_msgs::msg::Image right_msg;
    {
      std::lock_guard<std::mutex> lock(latest_frames_mutex_);
      if (!have_latest_frames_) {
        return;
      }
      left_msg = latest_left_msg_;
      right_msg = latest_right_msg_;
    }

    left_msg.header.stamp = now();
    right_msg.header.stamp = left_msg.header.stamp;
    left_throttled_pub_->publish(left_msg);
    right_throttled_pub_->publish(right_msg);
  }

  double acquisition_rate_hz_{20.0};
  double throttled_rate_hz_{1.0};
  int camera_width_{1280};
  int camera_height_{720};
  std::string frame_id_;
  std::string left_topic_;
  std::string right_topic_;

  cv::VideoCapture cam0_;
  cv::VideoCapture cam1_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_throttled_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_throttled_pub_;
  rclcpp::TimerBase::SharedPtr acquisition_timer_;
  rclcpp::TimerBase::SharedPtr throttled_timer_;

  std::mutex latest_frames_mutex_;
  sensor_msgs::msg::Image latest_left_msg_;
  sensor_msgs::msg::Image latest_right_msg_;
  bool have_latest_frames_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<StereoCameraNode>());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("stereo_camera"), "Exception: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
