#include "radiometric_processing_cpp/fast_radiometric_processing.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <custom_msgs/msg/hyperspectral_image.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

using radiometric_processing_cpp::FastProcessPipeline;
using radiometric_processing_cpp::resolve_context;

class RadiometricProcessingNode : public rclcpp::Node
{
public:
  RadiometricProcessingNode()
  : Node("radiometric_processing")
  {
    declare_parameter<std::string>("device_name", "");
    declare_parameter<std::string>("run_name_path", "");
    declare_parameter<std::string>("mission_state_topic", "/mission_state");
    declare_parameter<bool>("publish_rgb", true);
    declare_parameter<bool>("publish_cube", true);
    declare_parameter<double>("throttled_rate_hz", 1.0);
    declare_parameter<std::string>("mission_state", "Idle");

    device_name_ = get_parameter("device_name").as_string();
    const std::string run_name_path = get_parameter("run_name_path").as_string();
    const std::string raw_image_topic = "/hsi/" + device_name_ + "/raw";
    const std::string output_topic = "/hsi/" + device_name_ + "/reflectance";
    const std::string mission_state_topic = get_parameter("mission_state_topic").as_string();
    mission_state_ = get_parameter("mission_state").as_string();
    publish_rgb_ = get_parameter("publish_rgb").as_bool();
    publish_cube_ = get_parameter("publish_cube").as_bool();
    throttled_rate_hz_ = get_parameter("throttled_rate_hz").as_double();
    if (throttled_rate_hz_ <= 0.0) {
      throw std::runtime_error("throttled_rate_hz must be > 0");
    }

    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.best_effort();

    reflectance_pub_ = create_publisher<custom_msgs::msg::HyperspectralImage>(output_topic, 10);
    reflectance_throttled_pub_ = create_publisher<custom_msgs::msg::HyperspectralImage>(
      output_topic + "/throttled", 10);
    rgb_pub_ = create_publisher<sensor_msgs::msg::Image>(output_topic + "/rgb", 10);
    mission_sub_ = create_subscription<std_msgs::msg::String>(
      mission_state_topic, 10,
      std::bind(&RadiometricProcessingNode::on_mission_input, this, std::placeholders::_1));
    raw_sub_ = create_subscription<sensor_msgs::msg::Image>(
      raw_image_topic, qos,
      std::bind(&RadiometricProcessingNode::on_raw_input, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / throttled_rate_hz_);
    throttled_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&RadiometricProcessingNode::throttled_metronome, this));

    context_path_ = run_name_path + "/" + device_name_ + "/raw/context";
    RCLCPP_INFO(
      get_logger(),
      "%s radiometric processing (cpp) ready (throttled_rate_hz=%.3f) context=%s",
      device_name_.c_str(), throttled_rate_hz_, context_path_.c_str());
  }

private:
  void throttled_metronome()
  {
    throttled_tick_ = true;
  }

  void on_mission_input(const std_msgs::msg::String & msg)
  {
    mission_state_ = msg.data;
  }

  void on_raw_input(const sensor_msgs::msg::Image & msg)
  {
    if (!publish_cube_ && !publish_rgb_) {
      return;
    }

    if (pipeline_active_) {
      if (msg.encoding != "mono16" && msg.encoding != "16UC1") {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "unexpected encoding '%s' (expected mono16)", msg.encoding.c_str());
      }
      // Wrap message buffer as Mat (no copy), then convertTo float32.
      const cv::Mat raw_u16(
        static_cast<int>(msg.height), static_cast<int>(msg.width), CV_16UC1,
        const_cast<uint8_t *>(msg.data.data()), static_cast<size_t>(msg.step));
      cv::Mat raw_f32;
      raw_u16.convertTo(raw_f32, CV_32FC1);

      const cv::Mat reflectance_cube = fpp_->fast_process_to_reflectance(raw_f32);
      publish_rgb_preview(reflectance_cube);
      publish_hypercube(reflectance_cube);
      return;
    }

    if (context_complete_) {
      try {
        fpp_ = std::make_unique<FastProcessPipeline>(context_path_, "hsi_irradiance");
        pipeline_active_ = true;
        RCLCPP_INFO(get_logger(), "%s Pipeline Active", device_name_.c_str());
      } catch (const std::exception & ex) {
        RCLCPP_ERROR(get_logger(), "Pipeline init failed: %s", ex.what());
      }
      return;
    }

    try {
      RCLCPP_INFO(get_logger(), "%s checking %s", device_name_.c_str(), context_path_.c_str());
      (void)resolve_context(context_path_);
      context_complete_ = true;
      RCLCPP_INFO(get_logger(), "%s Context Ready", device_name_.c_str());
    } catch (const std::exception & ex) {
      RCLCPP_INFO(get_logger(), "Context Not Ready: %s", ex.what());
    }
  }

  void publish_rgb_preview(const cv::Mat & cube)
  {
    if (!publish_rgb_) {
      return;
    }
    const int bands = cube.channels();
    int idx_r = 14;
    int idx_g = 7;
    int idx_b = 2;
    if (device_name_ == "nir") {
      idx_r = 20;
      idx_g = 7;
      idx_b = 2;
    }
    if (idx_r >= bands || idx_g >= bands || idx_b >= bands) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "RGB band indices out of range for %d bands", bands);
      return;
    }

    const int out_h = cube.rows / binning_factor_;
    const int out_w = cube.cols / binning_factor_;
    if (out_h <= 0 || out_w <= 0) {
      return;
    }

    std::vector<cv::Mat> chans;
    cv::split(cube, chans);

    cv::Mat r_f, g_f, b_f;
    const cv::Size out_size(out_w, out_h);
    // Same as Python cube[::bin, ::bin, band] — nearest subsample.
    cv::resize(chans[static_cast<size_t>(idx_r)], r_f, out_size, 0, 0, cv::INTER_NEAREST);
    cv::resize(chans[static_cast<size_t>(idx_g)], g_f, out_size, 0, 0, cv::INTER_NEAREST);
    cv::resize(chans[static_cast<size_t>(idx_b)], b_f, out_size, 0, 0, cv::INTER_NEAREST);

    cv::Mat rgb_f;
    cv::merge(std::vector<cv::Mat>{r_f, g_f, b_f}, rgb_f);
    cv::Mat rgb_u8;
    // Saturating cast [0,1] → uint8 via scale 255.
    rgb_f.convertTo(rgb_u8, CV_8UC3, 255.0);

    sensor_msgs::msg::Image msg;
    msg.header.stamp = now();
    msg.header.frame_id = "hsi_" + device_name_ + "_reflectance_rgb";
    msg.height = static_cast<uint32_t>(rgb_u8.rows);
    msg.width = static_cast<uint32_t>(rgb_u8.cols);
    msg.encoding = "rgb8";
    msg.is_bigendian = 0;
    msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(rgb_u8.step);
    msg.data.assign(rgb_u8.datastart, rgb_u8.dataend);
    rgb_pub_->publish(msg);
  }

  void publish_hypercube(const cv::Mat & cube)
  {
    if (!publish_cube_) {
      return;
    }

    // Clip to [0,1] then scale to uint16 with OpenCV (keeps channel layout / interleave).
    cv::Mat clipped;
    cv::max(cube, 0.0, clipped);
    cv::min(clipped, 1.0, clipped);

    cv::Mat cube_u16;
    clipped.convertTo(cube_u16, CV_MAKETYPE(CV_16U, cube.channels()), 65535.0);
    CV_Assert(cube_u16.isContinuous());

    custom_msgs::msg::HyperspectralImage msg;
    msg.header.stamp = now();
    msg.header.frame_id = "hsi_" + device_name_ + "_reflectance";
    msg.height = static_cast<uint32_t>(cube_u16.rows);
    msg.width = static_cast<uint32_t>(cube_u16.cols);
    msg.bands = static_cast<uint32_t>(cube_u16.channels());
    msg.wavelengths_nm = fpp_->wavelengths_nm();

    const auto * begin = reinterpret_cast<const uint16_t *>(cube_u16.datastart);
    const auto * end = reinterpret_cast<const uint16_t *>(cube_u16.dataend);
    msg.data.assign(begin, end);

    reflectance_pub_->publish(msg);
    if (throttled_tick_) {
      reflectance_throttled_pub_->publish(msg);
      throttled_tick_ = false;
    }
  }

  std::string device_name_;
  std::string context_path_;
  std::string mission_state_;
  bool publish_rgb_{true};
  bool publish_cube_{true};
  double throttled_rate_hz_{1.0};
  int binning_factor_{2};

  bool throttled_tick_{false};
  bool pipeline_active_{false};
  bool context_complete_{false};
  std::unique_ptr<FastProcessPipeline> fpp_;

  rclcpp::Publisher<custom_msgs::msg::HyperspectralImage>::SharedPtr reflectance_pub_;
  rclcpp::Publisher<custom_msgs::msg::HyperspectralImage>::SharedPtr reflectance_throttled_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr raw_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_sub_;
  rclcpp::TimerBase::SharedPtr throttled_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<RadiometricProcessingNode>());
  } catch (const std::exception & ex) {
    fprintf(stderr, "radiometric_processing_node failed: %s\n", ex.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
