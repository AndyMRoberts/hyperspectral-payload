#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>

namespace
{

constexpr int kStaleLineThicknessPx = 20;

std::string trim_copy(std::string s)
{
  const auto not_space = [](unsigned char c) { return !std::isspace(c); };
  s.erase(s.begin(), std::find_if(s.begin(), s.end(), not_space));
  s.erase(std::find_if(s.rbegin(), s.rend(), not_space).base(), s.end());
  return s;
}

std::string to_lower_copy(std::string s)
{
  for (auto & c : s) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  return s;
}

cv::Mat read_matrix_or_throw(cv::FileStorage & fs, const std::string & key)
{
  cv::Mat matrix;
  fs[key] >> matrix;
  if (matrix.empty()) {
    throw std::runtime_error("Missing or empty calibration key: " + key);
  }
  return matrix;
}

cv::Mat image_to_gray_mat(const sensor_msgs::msg::Image & img)
{
  if (img.encoding == "mono8") {
    return cv::Mat(
      static_cast<int>(img.height), static_cast<int>(img.width), CV_8UC1,
      const_cast<uint8_t *>(img.data.data()), img.step).clone();
  }
  if (img.encoding == "bgr8") {
    cv::Mat bgr(
      static_cast<int>(img.height), static_cast<int>(img.width), CV_8UC3,
      const_cast<uint8_t *>(img.data.data()), img.step);
    cv::Mat gray;
    cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
    return gray;
  }
  if (img.encoding == "rgb8") {
    cv::Mat rgb(
      static_cast<int>(img.height), static_cast<int>(img.width), CV_8UC3,
      const_cast<uint8_t *>(img.data.data()), img.step);
    cv::Mat gray;
    cv::cvtColor(rgb, gray, cv::COLOR_RGB2GRAY);
    return gray;
  }
  throw std::runtime_error("unsupported encoding: " + img.encoding);
}

void mat_to_image_msg(
  const cv::Mat & mat, const std::string & encoding,
  const std_msgs::msg::Header & header, sensor_msgs::msg::Image & out)
{
  out.header = header;
  out.height = static_cast<uint32_t>(mat.rows);
  out.width = static_cast<uint32_t>(mat.cols);
  out.encoding = encoding;
  out.is_bigendian = false;
  out.step = static_cast<sensor_msgs::msg::Image::_step_type>(mat.cols * mat.elemSize());
  out.data.resize(static_cast<size_t>(mat.rows) * out.step);

  if (mat.isContinuous()) {
    std::memcpy(out.data.data(), mat.data, out.data.size());
  } else {
    for (int row = 0; row < mat.rows; ++row) {
      const auto row_bytes = static_cast<size_t>(out.step);
      std::memcpy(out.data.data() + row * row_bytes, mat.ptr(row), row_bytes);
    }
  }
}

cv::Mat bin_average(const cv::Mat & src, int bin_size)
{
  const int out_w = src.cols / bin_size;
  const int out_h = src.rows / bin_size;
  if (out_w <= 0 || out_h <= 0) {
    return {};
  }
  cv::Mat dst;
  cv::resize(src, dst, cv::Size(out_w, out_h), 0.0, 0.0, cv::INTER_AREA);
  return dst;
}

cv::Mat bin_max(const cv::Mat & src, int bin_size)
{
  const int out_w = src.cols / bin_size;
  const int out_h = src.rows / bin_size;
  if (out_w <= 0 || out_h <= 0) {
    return {};
  }

  const int dst_type = src.channels() == 3 ? CV_8UC3 : CV_8UC1;
  cv::Mat dst;
  dst.create(out_h, out_w, dst_type);

  const int channels = src.channels();
  for (int by = 0; by < out_h; ++by) {
    for (int bx = 0; bx < out_w; ++bx) {
      const int x0 = bx * bin_size;
      const int y0 = by * bin_size;
      if (channels == 1) {
        uint8_t mx = 0;
        for (int y = y0; y < y0 + bin_size; ++y) {
          const uint8_t * row = src.ptr<uint8_t>(y);
          for (int x = x0; x < x0 + bin_size; ++x) {
            mx = std::max(mx, row[x]);
          }
        }
        dst.at<uint8_t>(by, bx) = mx;
      } else {
        cv::Vec3b mx(0, 0, 0);
        for (int y = y0; y < y0 + bin_size; ++y) {
          const cv::Vec3b * row = src.ptr<cv::Vec3b>(y);
          for (int x = x0; x < x0 + bin_size; ++x) {
            mx[0] = std::max(mx[0], row[x][0]);
            mx[1] = std::max(mx[1], row[x][1]);
            mx[2] = std::max(mx[2], row[x][2]);
          }
        }
        dst.at<cv::Vec3b>(by, bx) = mx;
      }
    }
  }
  return dst;
}

void draw_stale_signal_mono8(sensor_msgs::msg::Image & out, int w, int h)
{
  out.height = static_cast<uint32_t>(h);
  out.width = static_cast<uint32_t>(w);
  out.encoding = "mono8";
  out.is_bigendian = false;
  out.step = static_cast<sensor_msgs::msg::Image::_step_type>(w);
  cv::Mat img = cv::Mat::zeros(h, w, CV_8UC1);
  const int t = std::min(kStaleLineThicknessPx, h);
  const int y0 = (h - t) / 2;
  cv::rectangle(img, cv::Rect(0, y0, w, t), cv::Scalar(255), cv::FILLED);
  out.data.assign(img.datastart, img.dataend);
}

}  // namespace

class StereoBinnedPreviewNode : public rclcpp::Node
{
public:
  enum class BinMode
  {
    kAverage,
    kMax
  };

  StereoBinnedPreviewNode()
  : Node("stereo_binned_preview")
  {
    declare_parameter<std::string>("left_topic", "/sensors/stereo/left");
    declare_parameter<std::string>("right_topic", "/sensors/stereo/right");
    declare_parameter<std::string>("topic_prefix", "/sensors/stereo");
    declare_parameter<double>("publish_rate_hz", 10.0);
    declare_parameter<int>("bin_size", 4);
    declare_parameter<std::string>("bin_mode", "average");
    declare_parameter<int>("camera_width", 1280);
    declare_parameter<int>("camera_height", 720);
    declare_parameter<double>("calibration_scale", 1.0);
    declare_parameter<int>("num_disparities", 16 * 5);
    declare_parameter<int>("sad_window_size", 21);
    declare_parameter<std::string>("mission_state_topic", "/mission_state");
    declare_parameter<double>("max_input_idle_s", 0.5);

    const std::string default_cal_dir = get_default_calibration_dir();
    declare_parameter<std::string>(
      "intrinsics_file",
      default_cal_dir.empty() ? "" : default_cal_dir + "/intrinsics.yml");
    declare_parameter<std::string>(
      "extrinsics_file",
      default_cal_dir.empty() ? "" : default_cal_dir + "/extrinsics.yml");

    left_topic_ = get_parameter("left_topic").as_string();
    right_topic_ = get_parameter("right_topic").as_string();
    topic_prefix_ = trim_copy(get_parameter("topic_prefix").as_string());
    if (topic_prefix_.empty()) {
      topic_prefix_ = "/sensors/stereo";
    }
    publish_rate_hz_ = get_parameter("publish_rate_hz").as_double();
    bin_size_ = get_parameter("bin_size").as_int();
    bin_mode_ = parse_bin_mode(get_parameter("bin_mode").as_string());
    camera_width_ = get_parameter("camera_width").as_int();
    camera_height_ = get_parameter("camera_height").as_int();
    calibration_scale_ = get_parameter("calibration_scale").as_double();
    num_disparities_ = get_parameter("num_disparities").as_int();
    sad_window_size_ = get_parameter("sad_window_size").as_int();
    mission_state_topic_ = get_parameter("mission_state_topic").as_string();
    max_input_idle_s_ = get_parameter("max_input_idle_s").as_double();
    intrinsics_file_ = get_parameter("intrinsics_file").as_string();
    extrinsics_file_ = get_parameter("extrinsics_file").as_string();

    if (camera_width_ <= 0 || camera_height_ <= 0) {
      throw std::runtime_error("camera_width and camera_height must be > 0");
    }
    if (calibration_scale_ <= 0.0) {
      throw std::runtime_error("calibration_scale must be > 0");
    }
    if (num_disparities_ < 16 || num_disparities_ % 16 != 0) {
      throw std::runtime_error("num_disparities must be a positive integer divisible by 16");
    }
    if (sad_window_size_ < 1 || sad_window_size_ % 2 != 1) {
      throw std::runtime_error("sad_window_size must be a positive odd number");
    }
    clamp_bin_size();

    load_calibration();
    init_stereo_matcher();

    rclcpp::QoS qos_sub(rclcpp::KeepLast(1));
    qos_sub.best_effort();
    rclcpp::QoS qos_pub(rclcpp::KeepLast(1));
    qos_pub.best_effort();

    left_sub_ = create_subscription<sensor_msgs::msg::Image>(
      left_topic_, qos_sub,
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_left_ = std::move(msg);
        last_left_rx_ = std::chrono::steady_clock::now();
      });
    right_sub_ = create_subscription<sensor_msgs::msg::Image>(
      right_topic_, qos_sub,
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_right_ = std::move(msg);
        last_right_rx_ = std::chrono::steady_clock::now();
      });

    mission_sub_ = create_subscription<std_msgs::msg::String>(
      mission_state_topic_, rclcpp::QoS(10),
      [this](const std_msgs::msg::String & msg) {
        if (to_lower_copy(trim_copy(msg.data)) == "shutdown") {
          RCLCPP_INFO(get_logger(), "mission_state shutdown — exiting cleanly");
          rclcpp::shutdown();
        }
      });

    left_pub_ = create_publisher<sensor_msgs::msg::Image>(topic_prefix_ + "/preview/left", qos_pub);
    right_pub_ = create_publisher<sensor_msgs::msg::Image>(topic_prefix_ + "/preview/right", qos_pub);
    stereo_pub_ = create_publisher<sensor_msgs::msg::Image>(topic_prefix_ + "/preview/stereo", qos_pub);

    const double period = publish_rate_hz_ > 0.0 ? (1.0 / publish_rate_hz_) : 0.1;
    timer_ = create_wall_timer(
      std::chrono::duration<double>(period),
      std::bind(&StereoBinnedPreviewNode::on_timer, this));

    RCLCPP_INFO(
      get_logger(),
      "Stereo binned preview: raw left=%s right=%s -> %s/preview/{left,right,stereo}, "
      "bin_size=%d, bin_mode=%s, rate=%.3f Hz, size=%dx%d, "
      "num_disparities=%d, sad_window_size=%d",
      left_topic_.c_str(), right_topic_.c_str(), topic_prefix_.c_str(),
      bin_size_, bin_mode_to_string(bin_mode_).c_str(), publish_rate_hz_,
      camera_width_, camera_height_, num_disparities_, sad_window_size_);
    if (use_calibration_) {
      RCLCPP_INFO(
        get_logger(),
        "Rectification enabled: intrinsics='%s' extrinsics='%s' scale=%.3f",
        intrinsics_file_.c_str(), extrinsics_file_.c_str(), calibration_scale_);
    } else {
      RCLCPP_WARN(get_logger(), "Rectification disabled (no calibration files loaded)");
    }
  }

private:
  static std::string get_default_calibration_dir()
  {
    try {
      return ament_index_cpp::get_package_share_directory("stereo_camera") + "/cal_results";
    } catch (const std::exception &) {
      return "";
    }
  }

  BinMode parse_bin_mode(const std::string & s) const
  {
    const std::string mode = to_lower_copy(trim_copy(s));
    if (mode == "max" || mode == "bin_max" || mode == "binned_max") {
      return BinMode::kMax;
    }
    if (mode == "average" || mode == "bin_average" || mode == "binned_average" || mode == "avg") {
      return BinMode::kAverage;
    }
    RCLCPP_WARN(
      get_logger(),
      "bin_mode '%s' is invalid; use 'average' or 'max' (defaulting to average)", s.c_str());
    return BinMode::kAverage;
  }

  std::string bin_mode_to_string(BinMode mode) const
  {
    return mode == BinMode::kMax ? "max" : "average";
  }

  void clamp_bin_size()
  {
    if (bin_size_ < 1) {
      bin_size_ = 1;
    }
  }

  void load_calibration()
  {
    const bool has_intrinsics = !intrinsics_file_.empty();
    const bool has_extrinsics = !extrinsics_file_.empty();
    if (!has_intrinsics && !has_extrinsics) {
      return;
    }
    if (has_intrinsics != has_extrinsics) {
      throw std::runtime_error(
        "Both intrinsics_file and extrinsics_file must be set, or neither");
    }

    cv::FileStorage intrinsics_fs(intrinsics_file_, cv::FileStorage::READ);
    if (!intrinsics_fs.isOpened()) {
      throw std::runtime_error("Failed to open intrinsics file: " + intrinsics_file_);
    }

    m1_ = read_matrix_or_throw(intrinsics_fs, "M1");
    d1_ = read_matrix_or_throw(intrinsics_fs, "D1");
    m2_ = read_matrix_or_throw(intrinsics_fs, "M2");
    d2_ = read_matrix_or_throw(intrinsics_fs, "D2");

    if (calibration_scale_ != 1.0) {
      m1_ *= calibration_scale_;
      m2_ *= calibration_scale_;
    }

    cv::FileStorage extrinsics_fs(extrinsics_file_, cv::FileStorage::READ);
    if (!extrinsics_fs.isOpened()) {
      throw std::runtime_error("Failed to open extrinsics file: " + extrinsics_file_);
    }

    r_ = read_matrix_or_throw(extrinsics_fs, "R");
    t_ = read_matrix_or_throw(extrinsics_fs, "T");

    const int rect_width = static_cast<int>(std::lround(camera_width_ * calibration_scale_));
    const int rect_height = static_cast<int>(std::lround(camera_height_ * calibration_scale_));
    const cv::Size image_size(rect_width, rect_height);
    cv::Mat q;
    cv::stereoRectify(
      m1_, d1_, m2_, d2_, image_size, r_, t_,
      r1_, r2_, p1_, p2_, q,
      cv::CALIB_ZERO_DISPARITY, -1, image_size, &roi1_, &roi2_);

    cv::initUndistortRectifyMap(
      m1_, d1_, r1_, p1_, image_size, CV_16SC2, map1_left_, map2_left_);
    cv::initUndistortRectifyMap(
      m2_, d2_, r2_, p2_, image_size, CV_16SC2, map1_right_, map2_right_);

    use_calibration_ = true;
  }

  void init_stereo_matcher()
  {
    stereo_bm_ = cv::StereoBM::create(num_disparities_, sad_window_size_);
    stereo_bm_->setPreFilterCap(31);
    stereo_bm_->setMinDisparity(0);
    stereo_bm_->setNumDisparities(num_disparities_);
    stereo_bm_->setTextureThreshold(10);
    stereo_bm_->setUniquenessRatio(15);
    stereo_bm_->setSpeckleWindowSize(100);
    stereo_bm_->setSpeckleRange(32);
    stereo_bm_->setDisp12MaxDiff(1);

    if (use_calibration_) {
      stereo_bm_->setROI1(roi1_);
      stereo_bm_->setROI2(roi2_);
    }
  }

  bool validate_raw_image(const sensor_msgs::msg::Image::ConstSharedPtr & img, const char * tag)
  {
    if (!img) {
      return false;
    }
    if (img->encoding != "mono8" && img->encoding != "rgb8" && img->encoding != "bgr8") {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "%s: expected mono8, rgb8, or bgr8, got '%s'", tag, img->encoding.c_str());
      return false;
    }
    if (img->data.size() < img->step * img->height) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s: image data too small", tag);
      return false;
    }
    return true;
  }

  cv::Mat to_gray_and_scale(const sensor_msgs::msg::Image & img)
  {
    cv::Mat gray = image_to_gray_mat(img);
    if (calibration_scale_ != 1.0) {
      const int interpolation = calibration_scale_ < 1.0 ? cv::INTER_AREA : cv::INTER_CUBIC;
      cv::Mat resized;
      cv::resize(gray, resized, cv::Size(), calibration_scale_, calibration_scale_, interpolation);
      gray = resized;
    }
    return gray;
  }

  cv::Mat rectify_left(const cv::Mat & gray)
  {
    if (!use_calibration_) {
      return gray;
    }
    cv::Mat rect;
    cv::remap(gray, rect, map1_left_, map2_left_, cv::INTER_LINEAR);
    return rect;
  }

  cv::Mat rectify_right(const cv::Mat & gray)
  {
    if (!use_calibration_) {
      return gray;
    }
    cv::Mat rect;
    cv::remap(gray, rect, map1_right_, map2_right_, cv::INTER_LINEAR);
    return rect;
  }

  cv::Mat compute_disparity_8u(const cv::Mat & img_left, const cv::Mat & img_right)
  {
    cv::Mat disparity_16s(img_left.rows, img_left.cols, CV_16S);
    cv::Mat disparity_8u(img_left.rows, img_left.cols, CV_8UC1);
    stereo_bm_->compute(img_left, img_right, disparity_16s);

    double min_val = 0.0;
    double max_val = 0.0;
    cv::minMaxLoc(disparity_16s, &min_val, &max_val);
    const double scale = (max_val > min_val) ? (255.0 / (max_val - min_val)) : 0.0;
    disparity_16s.convertTo(disparity_8u, CV_8UC1, scale);
    return disparity_8u;
  }

  cv::Mat bin_mat(const cv::Mat & src, BinMode mode)
  {
    if (src.empty()) {
      return {};
    }
    return (mode == BinMode::kMax) ? bin_max(src, bin_size_) : bin_average(src, bin_size_);
  }

  void publish_stale_mono(
    const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr & pub, int w, int h)
  {
    if (w <= 0 || h <= 0) {
      return;
    }
    const int preview_w = std::max(1, w / bin_size_);
    const int preview_h = std::max(1, h / bin_size_);
    std_msgs::msg::Header hdr;
    hdr.stamp = now();
    hdr.frame_id = "stale_input";
    sensor_msgs::msg::Image out;
    out.header = hdr;
    draw_stale_signal_mono8(out, preview_w, preview_h);
    pub->publish(out);
  }

  void on_timer()
  {
    bin_size_ = get_parameter("bin_size").as_int();
    max_input_idle_s_ = get_parameter("max_input_idle_s").as_double();
    clamp_bin_size();

    sensor_msgs::msg::Image::ConstSharedPtr left_raw;
    sensor_msgs::msg::Image::ConstSharedPtr right_raw;
    std::chrono::steady_clock::time_point left_rx;
    std::chrono::steady_clock::time_point right_rx;
    BinMode mode = BinMode::kAverage;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      left_raw = last_left_;
      right_raw = last_right_;
      left_rx = last_left_rx_;
      right_rx = last_right_rx_;
      mode = bin_mode_;
    }

    const std::string mode_param = to_lower_copy(trim_copy(get_parameter("bin_mode").as_string()));
    if (mode_param == "max" || mode_param == "bin_max" || mode_param == "binned_max") {
      mode = BinMode::kMax;
    } else if (
      mode_param == "average" || mode_param == "bin_average" ||
      mode_param == "binned_average" || mode_param == "avg")
    {
      mode = BinMode::kAverage;
    }
    bin_mode_ = mode;

    const auto steady_now = std::chrono::steady_clock::now();
    const double left_idle_s = std::chrono::duration<double>(steady_now - left_rx).count();
    const double right_idle_s = std::chrono::duration<double>(steady_now - right_rx).count();
    const bool input_stale =
      max_input_idle_s_ > 0.0 &&
      (left_idle_s > max_input_idle_s_ || right_idle_s > max_input_idle_s_);

    if (input_stale) {
      publish_stale_mono(left_pub_, last_preview_w_, last_preview_h_);
      publish_stale_mono(right_pub_, last_preview_w_, last_preview_h_);
      publish_stale_mono(stereo_pub_, last_preview_w_, last_preview_h_);
      return;
    }

    if (!validate_raw_image(left_raw, "left") || !validate_raw_image(right_raw, "right")) {
      return;
    }

    try {
      cv::Mat img_left = rectify_left(to_gray_and_scale(*left_raw));
      cv::Mat img_right = rectify_right(to_gray_and_scale(*right_raw));
      if (img_left.empty() || img_right.empty()) {
        return;
      }

      const cv::Mat disparity_8u = compute_disparity_8u(img_left, img_right);
      const cv::Mat preview_left = bin_mat(img_left, mode);
      const cv::Mat preview_right = bin_mat(img_right, mode);
      const cv::Mat preview_stereo = bin_mat(disparity_8u, mode);
      if (preview_left.empty() || preview_right.empty() || preview_stereo.empty()) {
        return;
      }

      last_preview_w_ = preview_left.cols;
      last_preview_h_ = preview_left.rows;

      std_msgs::msg::Header header = left_raw->header;
      header.stamp = now();

      sensor_msgs::msg::Image left_msg;
      sensor_msgs::msg::Image right_msg;
      sensor_msgs::msg::Image stereo_msg;
      mat_to_image_msg(preview_left, "mono8", header, left_msg);
      mat_to_image_msg(preview_right, "mono8", header, right_msg);
      mat_to_image_msg(preview_stereo, "mono8", header, stereo_msg);

      left_pub_->publish(left_msg);
      right_pub_->publish(right_msg);
      stereo_pub_->publish(stereo_msg);
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "preview processing failed: %s", e.what());
    }
  }

  double publish_rate_hz_{10.0};
  int bin_size_{4};
  int camera_width_{1280};
  int camera_height_{720};
  double calibration_scale_{1.0};
  int num_disparities_{80};
  int sad_window_size_{21};
  BinMode bin_mode_{BinMode::kAverage};
  double max_input_idle_s_{0.5};
  bool use_calibration_{false};
  std::string mission_state_topic_;
  std::string left_topic_;
  std::string right_topic_;
  std::string topic_prefix_;
  std::string intrinsics_file_;
  std::string extrinsics_file_;

  int last_preview_w_{0};
  int last_preview_h_{0};

  cv::Mat m1_;
  cv::Mat d1_;
  cv::Mat m2_;
  cv::Mat d2_;
  cv::Mat r_;
  cv::Mat t_;
  cv::Mat r1_;
  cv::Mat r2_;
  cv::Mat p1_;
  cv::Mat p2_;
  cv::Rect roi1_;
  cv::Rect roi2_;
  cv::Mat map1_left_;
  cv::Mat map2_left_;
  cv::Mat map1_right_;
  cv::Mat map2_right_;

  cv::Ptr<cv::StereoBM> stereo_bm_;

  std::mutex mutex_;
  sensor_msgs::msg::Image::ConstSharedPtr last_left_;
  sensor_msgs::msg::Image::ConstSharedPtr last_right_;
  std::chrono::steady_clock::time_point last_left_rx_{};
  std::chrono::steady_clock::time_point last_right_rx_{};

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr right_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_sub_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr stereo_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<StereoBinnedPreviewNode>());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("stereo_binned_preview"), "Exception: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
