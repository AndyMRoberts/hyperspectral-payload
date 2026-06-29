#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include "hsi_binned_preview_cpp/srv/set_preview_type.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
  /** Row-major band index within one mosaic cell: band 0 = top-left, +1 = one step right, next row at +mosaic_size. */
constexpr int kVisBaseBin = 4;
constexpr int kNirBaseBin = 5;


// bands used for RGB images
constexpr int kVisBandRed = 13;
constexpr int kVisBandGreen = 9;
constexpr int kVisBandBlue = 3;
constexpr int kNirBandRed = 19;
constexpr int kNirBandGreen = 8;
constexpr int kNirBandBlue = 2;


// helper function, trim whitespace
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

// get pixel value at location x and y in uint16 format
uint16_t read_mono16(const sensor_msgs::msg::Image & img, int x, int y)
{
  const size_t off = static_cast<size_t>(y) * img.step + static_cast<size_t>(x) * 2U;
  uint16_t v = 0;
  std::memcpy(&v, &img.data[off], sizeof(v));
  return v;
}

// write pixel value to image
void write_mono16(sensor_msgs::msg::Image & img, int x, int y, uint16_t v)
{
  const size_t off = static_cast<size_t>(y) * img.step + static_cast<size_t>(x) * 2U;
  std::memcpy(&img.data[off], &v, sizeof(v));
}

/** Stage 1 calibration path: max over each sensor mosaic block (4x4 VIS or 5x5 NIR). */
void max_bin_mosaic(
  const sensor_msgs::msg::Image & in, int mosaic_size,
  std::vector<uint16_t> & out_max, int & out_w, int & out_h)
{
  const int iw = static_cast<int>(in.width);
  const int ih = static_cast<int>(in.height);
  out_w = iw / mosaic_size;
  out_h = ih / mosaic_size;
  const size_t cells = static_cast<size_t>(out_w) * static_cast<size_t>(out_h);
  out_max.assign(cells, 0);

  for (int by = 0; by < out_h; ++by) {
    for (int bx = 0; bx < out_w; ++bx) {
      const int x0 = bx * mosaic_size;
      const int y0 = by * mosaic_size;
      uint16_t mx = 0;
      for (int y = y0; y < y0 + mosaic_size; ++y) {
        for (int x = x0; x < x0 + mosaic_size; ++x) {
          mx = std::max(mx, read_mono16(in, x, y));
        }
      }
      const size_t idx = static_cast<size_t>(by * out_w + bx);
      out_max[idx] = mx;
    }
  }
}

/** Non-calibration grayscale path: average over each sensor mosaic block. */
void average_bin_mosaic(
  const sensor_msgs::msg::Image & in, int mosaic_size,
  std::vector<uint16_t> & out_avg, int & out_w, int & out_h)
{
  const int iw = static_cast<int>(in.width);
  const int ih = static_cast<int>(in.height);
  out_w = iw / mosaic_size;
  out_h = ih / mosaic_size;
  const size_t cells = static_cast<size_t>(out_w) * static_cast<size_t>(out_h);
  out_avg.assign(cells, 0);

  for (int by = 0; by < out_h; ++by) {
    for (int bx = 0; bx < out_w; ++bx) {
      const int x0 = bx * mosaic_size;
      const int y0 = by * mosaic_size;
      uint64_t sum = 0;
      int count = 0;
      for (int y = y0; y < y0 + mosaic_size; ++y) {
        for (int x = x0; x < x0 + mosaic_size; ++x) {
          sum += static_cast<uint64_t>(read_mono16(in, x, y));
          ++count;
        }
      }
      const size_t idx = static_cast<size_t>(by * out_w + bx);
      out_avg[idx] = count > 0 ? static_cast<uint16_t>(sum / static_cast<uint64_t>(count)) : 0;
    }
  }
}

void subpixel_xy_from_band(int band, int mosaic_size, int & dx, int & dy)
{
  dx = band % mosaic_size;
  dy = band / mosaic_size;
}

/** RGB at mosaic resolution (H/mosaic, W/mosaic): R/G/B from raw subpixel band # in each cell. */
bool mosaic_bands_to_rgb8(
  const sensor_msgs::msg::Image & raw, int mosaic_size, int band_r, int band_g, int band_b,
  int extra_bin, sensor_msgs::msg::Image & out)
{
  const int nb = mosaic_size * mosaic_size;
  if (band_r < 0 || band_r >= nb || band_g < 0 || band_g >= nb || band_b < 0 || band_b >= nb) {
    return false;
  }

  int rdx = 0;
  int rdy = 0;
  int gdx = 0;
  int gdy = 0;
  int bdx = 0;
  int bdy = 0;
  subpixel_xy_from_band(band_r, mosaic_size, rdx, rdy);
  subpixel_xy_from_band(band_g, mosaic_size, gdx, gdy);
  subpixel_xy_from_band(band_b, mosaic_size, bdx, bdy);

  const int iw = static_cast<int>(raw.width);
  const int ih = static_cast<int>(raw.height);
  const int out_w = iw / mosaic_size;
  const int out_h = ih / mosaic_size;
  if (out_w <= 0 || out_h <= 0) {
    return false;
  }

  const int bin = std::max(extra_bin, 1);
  const int final_w = out_w / bin;
  const int final_h = out_h / bin;
  if (final_w <= 0 || final_h <= 0) {
    return false;
  }

  std::vector<uint16_t> r_cells(static_cast<size_t>(out_w) * static_cast<size_t>(out_h));
  std::vector<uint16_t> g_cells(static_cast<size_t>(out_w) * static_cast<size_t>(out_h));
  std::vector<uint16_t> b_cells(static_cast<size_t>(out_w) * static_cast<size_t>(out_h));

  for (int by = 0; by < out_h; ++by) {
    for (int bx = 0; bx < out_w; ++bx) {
      const int x0 = bx * mosaic_size;
      const int y0 = by * mosaic_size;
      const size_t idx = static_cast<size_t>(by * out_w + bx);
      r_cells[idx] = read_mono16(raw, x0 + rdx, y0 + rdy);
      g_cells[idx] = read_mono16(raw, x0 + gdx, y0 + gdy);
      b_cells[idx] = read_mono16(raw, x0 + bdx, y0 + bdy);
    }
  }

  std::vector<uint16_t> r_binned(static_cast<size_t>(final_w) * static_cast<size_t>(final_h));
  std::vector<uint16_t> g_binned(static_cast<size_t>(final_w) * static_cast<size_t>(final_h));
  std::vector<uint16_t> b_binned(static_cast<size_t>(final_w) * static_cast<size_t>(final_h));
  uint16_t min_r = 65535;
  uint16_t max_r = 0;
  uint16_t min_g = 65535;
  uint16_t max_g = 0;
  uint16_t min_b = 65535;
  uint16_t max_b = 0;

  for (int by = 0; by < final_h; ++by) {
    for (int bx = 0; bx < final_w; ++bx) {
      uint64_t rsum = 0;
      uint64_t gsum = 0;
      uint64_t bsum = 0;
      int count = 0;
      for (int y = by * bin; y < (by + 1) * bin; ++y) {
        for (int x = bx * bin; x < (bx + 1) * bin; ++x) {
          const size_t idx = static_cast<size_t>(y * out_w + x);
          rsum += static_cast<uint64_t>(r_cells[idx]);
          gsum += static_cast<uint64_t>(g_cells[idx]);
          bsum += static_cast<uint64_t>(b_cells[idx]);
          ++count;
        }
      }
      const uint16_t vr =
        count > 0 ? static_cast<uint16_t>(rsum / static_cast<uint64_t>(count)) : 0;
      const uint16_t vg =
        count > 0 ? static_cast<uint16_t>(gsum / static_cast<uint64_t>(count)) : 0;
      const uint16_t vb =
        count > 0 ? static_cast<uint16_t>(bsum / static_cast<uint64_t>(count)) : 0;
      const size_t oidx = static_cast<size_t>(by * final_w + bx);
      r_binned[oidx] = vr;
      g_binned[oidx] = vg;
      b_binned[oidx] = vb;
      min_r = std::min(min_r, vr);
      max_r = std::max(max_r, vr);
      min_g = std::min(min_g, vg);
      max_g = std::max(max_g, vg);
      min_b = std::min(min_b, vb);
      max_b = std::max(max_b, vb);
    }
  }

  const float dr = static_cast<float>(max_r > min_r ? (max_r - min_r) : 1);
  const float dg = static_cast<float>(max_g > min_g ? (max_g - min_g) : 1);
  const float db = static_cast<float>(max_b > min_b ? (max_b - min_b) : 1);

  out.height = static_cast<uint32_t>(final_h);
  out.width = static_cast<uint32_t>(final_w);
  out.encoding = "rgb8";
  out.is_bigendian = 0;
  out.step = static_cast<sensor_msgs::msg::Image::_step_type>(final_w * 3);
  out.data.resize(static_cast<size_t>(final_h) * out.step);

  for (int by = 0; by < final_h; ++by) {
    for (int bx = 0; bx < final_w; ++bx) {
      const size_t i = static_cast<size_t>(by * final_w + bx);
      const uint16_t vr = r_binned[i];
      const uint16_t vg = g_binned[i];
      const uint16_t vb = b_binned[i];
      const auto scale = [](uint16_t v, uint16_t lo, float d) -> uint8_t {
        const long x = std::lround((static_cast<float>(v - lo) / d) * 255.0f);
        return static_cast<uint8_t>(std::clamp(x, 0L, 255L));
      };
      const size_t o =
        static_cast<size_t>(by) * static_cast<size_t>(out.step) + static_cast<size_t>(bx) * 3U;
      out.data[o + 0] = scale(vr, min_r, dr);
      out.data[o + 1] = scale(vg, min_g, dg);
      out.data[o + 2] = scale(vb, min_b, db);
    }
  }
  return true;
}

// average binning for a grayscale image.
void bin_avg_extra(
  const std::vector<uint16_t> & in, int iw, int ih, int block,
  std::vector<uint16_t> & out_avg, int & out_w, int & out_h)
{
  out_w = iw / block;
  out_h = ih / block;
  if (out_w <= 0 || out_h <= 0) {
    out_avg.clear();
    return;
  }

  const cv::Mat in_mat(ih, iw, CV_16UC1, const_cast<uint16_t *>(in.data()));
  cv::Mat out_mat;
  // INTER_AREA gives area-averaged downsampling, matching block-average behavior.
  cv::resize(in_mat, out_mat, cv::Size(out_w, out_h), 0.0, 0.0, cv::INTER_AREA);

  out_avg.resize(static_cast<size_t>(out_w) * static_cast<size_t>(out_h));
  std::memcpy(out_avg.data(), out_mat.data, out_avg.size() * sizeof(uint16_t));
}

/** Max pool in blocks — keeps a hot / saturated pixel from being averaged down (used in calibration / overexposure). */
void bin_max_extra(
  const std::vector<uint16_t> & in, int iw, int ih, int block,
  std::vector<uint16_t> & out_max, int & out_w, int & out_h)
{
  out_w = iw / block;
  out_h = ih / block;
  const size_t cells = static_cast<size_t>(out_w) * static_cast<size_t>(out_h);
  out_max.assign(cells, 0);

  for (int by = 0; by < out_h; ++by) {
    for (int bx = 0; bx < out_w; ++bx) {
      const int x0 = bx * block;
      const int y0 = by * block;
      uint16_t mx = 0;
      for (int y = y0; y < y0 + block; ++y) {
        for (int x = x0; x < x0 + block; ++x) {
          mx = std::max(mx, in[static_cast<size_t>(y * iw + x)]);
        }
      }
      const size_t idx = static_cast<size_t>(by * out_w + bx);
      out_max[idx] = mx;
    }
  }
}

void fill_image_from_vector(
  const std::vector<uint16_t> & v, int ow, int oh, const std_msgs::msg::Header & header,
  sensor_msgs::msg::Image & out)
{
  out.header = header;
  out.height = static_cast<uint32_t>(oh);
  out.width = static_cast<uint32_t>(ow);
  out.encoding = "mono16";
  out.is_bigendian = 0;
  out.step = static_cast<sensor_msgs::msg::Image::_step_type>(ow * 2);
  out.data.resize(static_cast<size_t>(oh) * out.step);
  for (int y = 0; y < oh; ++y) {
    for (int x = 0; x < ow; ++x) {
      write_mono16(out, x, y, v[static_cast<size_t>(y * ow + x)]);
    }
  }
}

/** False-color for calibration: expects **same numeric range as /raw_image** (default driver uses 0..1023 in mono16).
 *  "Red" = near or at ADC top (clipped / overexposed if sample hit raw_mono_value_max on camera node).
 *  - bottom 1% of 10-bit: blue, top 1%: red, mid: grayscale
 */
void fill_falsecolor_rgb_from_vector_1024(
  const std::vector<uint16_t> & v, int ow, int oh, const std_msgs::msg::Header & header,
  sensor_msgs::msg::Image & out)
{
  constexpr uint16_t kRangeMax = 1023;
  constexpr uint16_t kBottom1Pct = 10;
  constexpr uint16_t kTop1Pct = 1013;

  out.header = header;
  out.height = static_cast<uint32_t>(oh);
  out.width = static_cast<uint32_t>(ow);
  out.encoding = "rgb8";
  out.is_bigendian = 0;
  out.step = static_cast<sensor_msgs::msg::Image::_step_type>(ow * 3);
  out.data.resize(static_cast<size_t>(oh) * out.step);

  for (int y = 0; y < oh; ++y) {
    for (int x = 0; x < ow; ++x) {
      const uint16_t raw = v[static_cast<size_t>(y * ow + x)];
      const uint16_t clamped = std::min<uint16_t>(raw, kRangeMax);
      const uint8_t gray = static_cast<uint8_t>((static_cast<uint32_t>(clamped) * 255U) / kRangeMax);

      uint8_t r = gray;
      uint8_t g = gray;
      uint8_t b = gray;

      if (clamped <= kBottom1Pct) {
        r = 0;
        g = 0;
        b = 255;
      } else if (clamped >= kTop1Pct) {
        r = 255;
        g = 0;
        b = 0;
      }

      const size_t o =
        static_cast<size_t>(y) * static_cast<size_t>(out.step) + static_cast<size_t>(x) * 3U;
      out.data[o + 0] = r;
      out.data[o + 1] = g;
      out.data[o + 2] = b;
    }
  }
}

constexpr int kStaleLineThicknessPx = 20;

/** Black image with a horizontal white band (20 px tall), vertically centered; full width. */
void draw_stale_signal_mono16(sensor_msgs::msg::Image & out, int w, int h)
{
  out.height = static_cast<uint32_t>(h);
  out.width = static_cast<uint32_t>(w);
  out.encoding = "mono16";
  out.is_bigendian = 0;
  out.step = static_cast<sensor_msgs::msg::Image::_step_type>(w * 2);
  cv::Mat img = cv::Mat::zeros(h, w, CV_16UC1);
  const int t = std::min(kStaleLineThicknessPx, h);
  const int y0 = (h - t) / 2;
  cv::rectangle(img, cv::Rect(0, y0, w, t), cv::Scalar(65535), cv::FILLED);
  out.data.assign(img.datastart, img.dataend);
}

void draw_stale_signal_rgb8(sensor_msgs::msg::Image & out, int w, int h)
{
  out.height = static_cast<uint32_t>(h);
  out.width = static_cast<uint32_t>(w);
  out.encoding = "rgb8";
  out.is_bigendian = 0;
  out.step = static_cast<sensor_msgs::msg::Image::_step_type>(w * 3);
  cv::Mat img = cv::Mat::zeros(h, w, CV_8UC3);
  const int t = std::min(kStaleLineThicknessPx, h);
  const int y0 = (h - t) / 2;
  cv::rectangle(img, cv::Rect(0, y0, w, t), cv::Scalar(255, 255, 255), cv::FILLED);
  out.data.assign(img.datastart, img.dataend);
}

}  // namespace

class HsiBinnedPreviewNode : public rclcpp::Node
{
public:
  enum class PreviewType
  {
    kBinMax,
    kBinAverage,
    kRgb
  };

  HsiBinnedPreviewNode():Node("hsi_binned_preview_node"){
    declare_parameter<std::string>("camera", "vis");
    declare_parameter<std::string>("raw_image_topic", "");
    declare_parameter<std::string>("topic_prefix", "");
    declare_parameter<double>("publish_rate_hz", 5.0);
    declare_parameter<int>("extra_bin", 1);
    /** Initial preview type: "bin_max", "bin_average", or "rgb". */
    declare_parameter<std::string>("preview_type", "rgb");
    declare_parameter<std::string>("mission_state_topic", "/mission_state");
    /** If no new raw image arrives for this long (seconds), publish stale sentinel images instead of repeating the last frame. Set <= 0 to disable (repeats last frame). */
    declare_parameter<double>("max_input_idle_s", 0.5);

    std::string camera = to_lower_copy(trim_copy(get_parameter("camera").as_string()));
    if (camera != "vis" && camera != "nir") {
      throw std::runtime_error(
        "hsi_binned_preview_node: parameter 'camera' must be 'vis' or 'nir' (got '" + camera + "')");
    }

    std::string raw_override = trim_copy(get_parameter("raw_image_topic").as_string());
    std::string topic_prefix = trim_copy(get_parameter("topic_prefix").as_string());
    if (topic_prefix.empty()) {
      topic_prefix = std::string("/hsi/") + camera;
    }

    if (raw_override.empty()) {
      raw_image_topic_ = std::string("/hsi/") + camera + "/raw";
    } else {
      raw_image_topic_ = std::move(raw_override);
    }

    publish_rate_hz_ = get_parameter("publish_rate_hz").as_double();
    extra_bin_ = get_parameter("extra_bin").as_int();
    preview_type_ = normalize_preview_type(get_parameter("preview_type").as_string());
    mission_state_topic_ = get_parameter("mission_state_topic").as_string();
    max_input_idle_s_ = get_parameter("max_input_idle_s").as_double();

    if (camera == "vis") {
      mosaic_size_ = kVisBaseBin;
      band_r_ = kVisBandRed;
      band_g_ = kVisBandGreen;
      band_b_ = kVisBandBlue;
    } else {
      mosaic_size_ = kNirBaseBin;
      band_r_ = kNirBandRed;
      band_g_ = kNirBandGreen;
      band_b_ = kNirBandBlue;
    }

    clamp_extra_bin();

    rclcpp::QoS qos_sub(rclcpp::KeepLast(1));
    qos_sub.best_effort();

    raw_sub_ = create_subscription<sensor_msgs::msg::Image>(
      raw_image_topic_, qos_sub,
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_raw_ = std::move(msg);
        last_image_rx_steady_ = std::chrono::steady_clock::now();
      });

    mission_sub_ = create_subscription<std_msgs::msg::String>(
      mission_state_topic_, rclcpp::QoS(10),
      [this](const std_msgs::msg::String & msg) {
        if (to_lower_copy(trim_copy(msg.data)) == "shutdown") {
          RCLCPP_INFO(get_logger(), "mission_state shutdown — exiting cleanly");
          rclcpp::shutdown();
        }
      });

    rclcpp::QoS qos_pub(rclcpp::KeepLast(1));
    qos_pub.best_effort();

    const std::string pfx = topic_prefix;
    preview_pub_ =
      create_publisher<sensor_msgs::msg::Image>(pfx + "/preview", qos_pub);

    set_preview_type_srv_ = create_service<hsi_binned_preview_cpp::srv::SetPreviewType>(
      "/hsi/set_preview_type",
      [this](
        const std::shared_ptr<hsi_binned_preview_cpp::srv::SetPreviewType::Request> req,
        std::shared_ptr<hsi_binned_preview_cpp::srv::SetPreviewType::Response> resp) {
        PreviewType parsed = PreviewType::kRgb;
        if (!try_parse_preview_type(req->mode, parsed)) {
          resp->success = false;
          resp->message = "invalid mode; use one of: bin_max, bin_average, rgb";
          RCLCPP_WARN(
            get_logger(), "Rejected preview type '%s': expected bin_max|bin_average|rgb",
            req->mode.c_str());
          return;
        }
        {
          std::lock_guard<std::mutex> lock(mutex_);
          preview_type_ = parsed;
        }
        resp->success = true;
        resp->message = "preview type set to " + preview_type_to_string(parsed);
        RCLCPP_INFO(get_logger(), "Preview type changed to '%s'", preview_type_to_string(parsed).c_str());
      });

    const double period = publish_rate_hz_ > 0.0 ? (1.0 / publish_rate_hz_) : 0.2;
    timer_ = create_wall_timer(
      std::chrono::duration<double>(period),
      std::bind(&HsiBinnedPreviewNode::on_timer, this));

    RCLCPP_INFO(
      get_logger(),
      "HSI binned preview [%s]: subscribe %s, publish %s/preview — mosaic %dx%d, extra_bin=%d, "
      "preview_type=%s, service=hsi/set_preview_type, rate=%.3f Hz, max_input_idle_s=%.3f "
      "(stale=sentinel)",
      camera.c_str(), raw_image_topic_.c_str(), pfx.c_str(), mosaic_size_, mosaic_size_, extra_bin_,
      preview_type_to_string(preview_type_).c_str(), publish_rate_hz_, max_input_idle_s_);
  }

private:
  bool try_parse_preview_type(const std::string & s, PreviewType & out) const
  {
    const std::string t = to_lower_copy(trim_copy(s));
    if (t == "bin_max" || t == "binned_max" || t == "max") {
      out = PreviewType::kBinMax;
      return true;
    }
    if (
      t == "bin_average" || t == "binned_average" || t == "average" ||
      t == "avg")
    {
      out = PreviewType::kBinAverage;
      return true;
    }
    if (t == "rgb") {
      out = PreviewType::kRgb;
      return true;
    }
    return false;
  }

  PreviewType normalize_preview_type(const std::string & s) const
  {
    PreviewType parsed = PreviewType::kRgb;
    if (!try_parse_preview_type(s, parsed)) {
      RCLCPP_WARN(
        get_logger(),
        "preview_type '%s' is invalid; use 'bin_max', 'bin_average', or 'rgb' (defaulting to rgb)",
        s.c_str());
      return PreviewType::kRgb;
    }
    return parsed;
  }

  std::string preview_type_to_string(PreviewType t) const
  {
    if (t == PreviewType::kBinMax) {
      return "bin_max";
    }
    if (t == PreviewType::kBinAverage) {
      return "bin_average";
    }
    return "rgb";
  }

  void clamp_extra_bin()
  {
    if (extra_bin_ < 1) {
      extra_bin_ = 1;
    }
  }

  void publish_stale_sentinels(){
    if (last_gray_w_ <= 0 || last_gray_h_ <= 0) {
      return;
    }
    std_msgs::msg::Header hdr;
    hdr.stamp = now();
    hdr.frame_id = "stale_input";
    sensor_msgs::msg::Image out;
    out.header = hdr;
    draw_stale_signal_mono16(out, last_gray_w_, last_gray_h_);
    preview_pub_->publish(out);
  }

  bool validate_mono16(const sensor_msgs::msg::Image::ConstSharedPtr & img, const char * tag)
  {
    if (!img) {
      return false;
    }
    if (img->encoding != "mono16") {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "%s: expected mono16, got '%s'", tag,
        img->encoding.c_str());
      return false;
    }
    if (img->data.size() < img->step * img->height) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s: image data too small", tag);
      return false;
    }
    return true;
  }

  void on_timer()
  {
    extra_bin_ = get_parameter("extra_bin").as_int();
    max_input_idle_s_ = get_parameter("max_input_idle_s").as_double();
    clamp_extra_bin();

    sensor_msgs::msg::Image::ConstSharedPtr raw;
    PreviewType preview_type_copy = PreviewType::kRgb;
    std::chrono::steady_clock::time_point last_rx_steady{};
    {
      std::lock_guard<std::mutex> lock(mutex_);
      raw = last_raw_;
      preview_type_copy = preview_type_;
      last_rx_steady = last_image_rx_steady_;
    }

    const auto steady_now = std::chrono::steady_clock::now();
    const double idle_s = std::chrono::duration<double>(steady_now - last_rx_steady).count();
    const bool input_stale =
      max_input_idle_s_ > 0.0 && (idle_s > max_input_idle_s_);

    if (input_stale) {
      publish_stale_sentinels();
      return;
    }

    if (!validate_mono16(raw, "raw")) {
      return;
    }

    const std_msgs::msg::Header hdr = raw->header;

    if (preview_type_copy == PreviewType::kBinMax) {
      std::vector<uint16_t> bmax;
      int cw = 0;
      int ch = 0;
      max_bin_mosaic(*raw, mosaic_size_, bmax, cw, ch);
      if (cw <= 0 || ch <= 0) {
        return;
      }

      int ow = cw;
      int oh = ch;
      if (extra_bin_ > 1) {
        std::vector<uint16_t> bmax2;
        // Max-pool: averaging after per-cell max would hide single-band saturation in a bin.
        bin_max_extra(bmax, cw, ch, extra_bin_, bmax2, ow, oh);
        bmax = std::move(bmax2);
      }
      if (ow <= 0 || oh <= 0) {
        return;
      }
      last_binned_w_ = ow;
      last_binned_h_ = oh;

      sensor_msgs::msg::Image out_falsecolor;
      fill_falsecolor_rgb_from_vector_1024(bmax, ow, oh, hdr, out_falsecolor);
      preview_pub_->publish(out_falsecolor);
    } else if (preview_type_copy == PreviewType::kBinAverage) {
      std::vector<uint16_t> collapsed;
      int cw = 0;
      int ch = 0;
      average_bin_mosaic(*raw, mosaic_size_, collapsed, cw, ch);
      if (cw <= 0 || ch <= 0) {
        return;
      }
      int ow = cw;
      int oh = ch;
      if (extra_bin_ > 1) {
        std::vector<uint16_t> bavg;
        bin_avg_extra(collapsed, cw, ch, extra_bin_, bavg, ow, oh);
        collapsed = std::move(bavg);
      }
      if (ow <= 0 || oh <= 0) {
        return;
      }
      last_gray_w_ = ow;
      last_gray_h_ = oh;
      sensor_msgs::msg::Image out_gray;
      fill_image_from_vector(collapsed, ow, oh, hdr, out_gray);
      preview_pub_->publish(out_gray);
    } else {
      sensor_msgs::msg::Image out_colour;
      out_colour.header = hdr;
      if (mosaic_bands_to_rgb8(
          *raw, mosaic_size_, band_r_, band_g_, band_b_, extra_bin_, out_colour))
      {
        last_colour_w_ = static_cast<int>(out_colour.width);
        last_colour_h_ = static_cast<int>(out_colour.height);
        preview_pub_->publish(out_colour);
      }
    }
  }

  double publish_rate_hz_{5.0};
  int extra_bin_{1};
  PreviewType preview_type_{PreviewType::kRgb};
  double max_input_idle_s_{1};
  std::string mission_state_topic_;
  std::string raw_image_topic_;
  int mosaic_size_{4};
  int band_r_{0};
  int band_g_{0};
  int band_b_{0};

  int last_binned_w_{0};
  int last_binned_h_{0};
  int last_colour_w_{0};
  int last_colour_h_{0};
  int last_gray_w_{0};
  int last_gray_h_{0};

  std::mutex mutex_;
  sensor_msgs::msg::Image::ConstSharedPtr last_raw_;
  std::chrono::steady_clock::time_point last_image_rx_steady_{};

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr raw_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_sub_;
  rclcpp::Service<hsi_binned_preview_cpp::srv::SetPreviewType>::SharedPtr set_preview_type_srv_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr preview_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HsiBinnedPreviewNode>());
  rclcpp::shutdown();
  return 0;
}
