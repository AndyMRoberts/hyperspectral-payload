#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

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
    << " ! video/x-raw, format=(string)I420"
    << " ! appsink drop=true max-buffers=1";
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
    single_mode_ = declare_parameter<bool>("single_mode", false);

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
    left_throttled_pub_ = create_publisher<sensor_msgs::msg::Image>(left_topic_ + "/throttled", qos);
    if (!single_mode_) {
      right_pub_ = create_publisher<sensor_msgs::msg::Image>(right_topic_, qos);
      right_throttled_pub_ = create_publisher<sensor_msgs::msg::Image>(
        right_topic_ + "/throttled", qos);
    }

    running_ = true;
    capture_thread_ = std::thread(&StereoCameraNode::capture_loop, this);

    const auto throttled_period = std::chrono::duration<double>(1.0 / throttled_rate_hz_);
    throttled_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(throttled_period),
      std::bind(&StereoCameraNode::throttled_callback, this));

    RCLCPP_INFO(
      get_logger(),
      "Stereo camera node started: single_mode=%s acquisition_rate_hz=%.3f throttled_rate_hz=%.3f "
      "size=%dx%d (cam0=right, cam1=left)",
      single_mode_ ? "true" : "false",
      acquisition_rate_hz_, throttled_rate_hz_, camera_width_, camera_height_);
    if (single_mode_) {
      RCLCPP_INFO(
        get_logger(),
        "Publishing raw left='%s' throttled left='%s/throttled' (right camera disabled)",
        left_topic_.c_str(), left_topic_.c_str());
    } else {
      RCLCPP_INFO(
        get_logger(),
        "Publishing raw left='%s' right='%s' throttled left='%s/throttled' right='%s/throttled'",
        left_topic_.c_str(), right_topic_.c_str(), left_topic_.c_str(), right_topic_.c_str());
    }
  }

  ~StereoCameraNode()
  {
    running_ = false;
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
    if (cam0_.isOpened()) {
      cam0_.release();
    }
    if (cam1_.isOpened()) {
      cam1_.release();
    }
  }

private:
  void open_cameras(int capture_fps)
  {
    // Waveshare IMX219-83: sensor-id 0 = right, sensor-id 1 = left
    if (!single_mode_) {
      const std::string cam0_pipeline = make_gstreamer_pipeline(
        0, camera_width_, camera_height_, capture_fps);
      cam0_.open(cam0_pipeline, cv::CAP_GSTREAMER);
      if (!cam0_.isOpened()) {
        throw std::runtime_error("cam0 is not opened (sensor-id=0, right)");
      }
      RCLCPP_INFO(get_logger(), "Opened cam0 (right)");
    }

    const std::string cam1_pipeline = make_gstreamer_pipeline(
      1, camera_width_, camera_height_, capture_fps);
    cam1_.open(cam1_pipeline, cv::CAP_GSTREAMER);
    if (!cam1_.isOpened()) {
      throw std::runtime_error("cam1 is not opened (sensor-id=1, left)");
    }
    RCLCPP_INFO(get_logger(), "Opened cam1 (left)");
  }

  void capture_loop()
  {
    cv::Mat frame_right_i420, frame_left_i420;
    cv::Mat frame_right_bgr, frame_left_bgr;

    while (running_ && rclcpp::ok()) {
      // Blocks until hardware delivers a frame (no timer drift)
      const bool grabbed_left = cam1_.grab();
      const bool grabbed_right = single_mode_ ? true : cam0_.grab();
      if (!grabbed_left || !grabbed_right) {
        continue;
      }

      if (!single_mode_) {
        cam0_.retrieve(frame_right_i420);
        if (frame_right_i420.empty()) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Empty frame from cam0");
          continue;
        }
      }

      cam1_.retrieve(frame_left_i420);
      if (frame_left_i420.empty()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Empty frame from cam1");
        continue;
      }

      cv::cvtColor(frame_left_i420, frame_left_bgr, cv::COLOR_YUV2BGR_I420);

      std_msgs::msg::Header header;
      header.stamp = now();
      header.frame_id = frame_id_;

      sensor_msgs::msg::Image left_msg;
      mat_to_image_msg(frame_left_bgr, "bgr8", header, left_msg);
      left_pub_->publish(left_msg);

      sensor_msgs::msg::Image right_msg;
      if (!single_mode_) {
        cv::cvtColor(frame_right_i420, frame_right_bgr, cv::COLOR_YUV2BGR_I420);
        mat_to_image_msg(frame_right_bgr, "bgr8", header, right_msg);
        right_pub_->publish(right_msg);
      }

      {
        std::lock_guard<std::mutex> lock(latest_frames_mutex_);
        latest_left_msg_ = left_msg;
        if (!single_mode_) {
          latest_right_msg_ = right_msg;
        }
        have_latest_frames_ = true;
      }
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
      if (!single_mode_) {
        right_msg = latest_right_msg_;
      }
    }

    left_msg.header.stamp = now();
    left_throttled_pub_->publish(left_msg);
    if (!single_mode_) {
      right_msg.header.stamp = left_msg.header.stamp;
      right_throttled_pub_->publish(right_msg);
    }
  }

  double acquisition_rate_hz_{20.0};
  double throttled_rate_hz_{1.0};
  int camera_width_{640};
  int camera_height_{480};
  std::string frame_id_;
  std::string left_topic_;
  std::string right_topic_;
  bool single_mode_{false};

  cv::VideoCapture cam0_;
  cv::VideoCapture cam1_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_throttled_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_throttled_pub_;
  rclcpp::TimerBase::SharedPtr throttled_timer_;

  std::thread capture_thread_;
  std::atomic<bool> running_{false};

  std::mutex latest_frames_mutex_;
  sensor_msgs::msg::Image latest_left_msg_;
  sensor_msgs::msg::Image latest_right_msg_;
  bool have_latest_frames_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<StereoCameraNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("stereo_camera"), "Exception: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
