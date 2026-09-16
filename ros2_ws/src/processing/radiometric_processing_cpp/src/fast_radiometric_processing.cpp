#include "radiometric_processing_cpp/fast_radiometric_processing.hpp"

#include <opencv2/imgproc.hpp>
#include <tinyxml2.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <sstream>
#include <stdexcept>

namespace fs = std::filesystem;

namespace radiometric_processing_cpp
{
namespace
{

fs::path first_glob(const fs::path & dir, const std::string & prefix, const std::string & suffix)
{
  if (!fs::exists(dir) || !fs::is_directory(dir)) {
    throw std::runtime_error("directory missing: " + dir.string());
  }
  std::vector<fs::path> matches;
  for (const auto & entry : fs::directory_iterator(dir)) {
    if (!entry.is_regular_file()) {
      continue;
    }
    const std::string name = entry.path().filename().string();
    if (name.size() < prefix.size() + suffix.size()) {
      continue;
    }
    if (name.compare(0, prefix.size(), prefix) != 0) {
      continue;
    }
    if (name.compare(name.size() - suffix.size(), suffix.size(), suffix) != 0) {
      continue;
    }
    matches.push_back(entry.path());
  }
  std::sort(matches.begin(), matches.end());
  if (matches.empty()) {
    throw std::runtime_error(
      "no match for " + prefix + "*" + suffix + " in " + dir.string());
  }
  return matches.front();
}

fs::path first_xml(const fs::path & dir)
{
  if (!fs::exists(dir) || !fs::is_directory(dir)) {
    throw std::runtime_error("directory missing: " + dir.string());
  }
  std::vector<fs::path> matches;
  for (const auto & entry : fs::directory_iterator(dir)) {
    if (!entry.is_regular_file()) {
      continue;
    }
    if (entry.path().extension() == ".xml") {
      matches.push_back(entry.path());
    }
  }
  std::sort(matches.begin(), matches.end());
  if (matches.empty()) {
    throw std::runtime_error("no *.xml in " + dir.string());
  }
  return matches.front();
}

std::vector<float> parse_float_list(const std::string & values)
{
  std::vector<float> out;
  std::istringstream iss(values);
  float v = 0.f;
  while (iss >> v) {
    out.push_back(v);
  }
  return out;
}

const tinyxml2::XMLElement * find_named(
  const tinyxml2::XMLElement * node, const char * name)
{
  if (node == nullptr) {
    return nullptr;
  }
  if (std::strcmp(node->Name(), name) == 0) {
    return node;
  }
  for (const auto * c = node->FirstChildElement(); c != nullptr; c = c->NextSiblingElement()) {
    if (const auto * found = find_named(c, name)) {
      return found;
    }
  }
  return nullptr;
}

}  // namespace

ContextPaths resolve_context(const fs::path & context_dir)
{
  ContextPaths ctx;
  ctx.calibration_xml = first_xml(context_dir / "calibration_file");
  ctx.white_non_uniformity_xml =
    first_glob(context_dir / "non_uniformity", "white_reference", ".raw.xml");
  ctx.dark_non_uniformity_xml =
    first_glob(context_dir / "non_uniformity", "dark_reference", ".raw.xml");
  ctx.white_reference_xml =
    first_glob(context_dir / "white_reference", "white_reference", ".raw.xml");

  const fs::path dark_dir = context_dir / "dark_references";
  if (fs::exists(dark_dir) && fs::is_directory(dark_dir)) {
    for (const auto & entry : fs::directory_iterator(dark_dir)) {
      if (!entry.is_regular_file()) {
        continue;
      }
      const std::string name = entry.path().filename().string();
      if (name.rfind("dark_reference", 0) == 0 &&
        name.size() >= 8 &&
        name.compare(name.size() - 8, 8, ".raw.xml") == 0)
      {
        ctx.dark_reference_xmls.push_back(entry.path());
      }
    }
    std::sort(ctx.dark_reference_xmls.begin(), ctx.dark_reference_xmls.end());
  }
  return ctx;
}

cv::Mat load_roi_frame(const fs::path & xml_path, float * integration_time_ms)
{
  tinyxml2::XMLDocument doc;
  if (doc.LoadFile(xml_path.string().c_str()) != tinyxml2::XML_SUCCESS) {
    throw std::runtime_error("failed to parse " + xml_path.string());
  }
  const auto * root = doc.RootElement();
  if (root == nullptr) {
    throw std::runtime_error("empty xml: " + xml_path.string());
  }
  const auto * fmt = root->FirstChildElement("data_format");
  const auto * info = root->FirstChildElement("data_info");
  if (fmt == nullptr || info == nullptr) {
    throw std::runtime_error("missing data_format/data_info in " + xml_path.string());
  }
  const int nr_rows = fmt->IntAttribute("nr_rows");
  const int nr_cols = fmt->IntAttribute("nr_cols");
  const float int_ms = static_cast<float>(info->DoubleAttribute("integration_time_ms"));
  if (integration_time_ms != nullptr) {
    *integration_time_ms = int_ms;
  }

  std::string raw_path = xml_path.string();
  const std::string suffix = ".raw.xml";
  if (raw_path.size() >= suffix.size() &&
    raw_path.compare(raw_path.size() - suffix.size(), suffix.size(), suffix) == 0)
  {
    raw_path.replace(raw_path.size() - suffix.size(), suffix.size(), ".raw");
  } else {
    throw std::runtime_error("expected .raw.xml path: " + xml_path.string());
  }

  std::ifstream in(raw_path, std::ios::binary);
  if (!in) {
    throw std::runtime_error("failed to open " + raw_path);
  }
  cv::Mat data(nr_rows, nr_cols, CV_32FC1);
  in.read(
    reinterpret_cast<char *>(data.data),
    static_cast<std::streamsize>(data.total() * data.elemSize()));
  if (!in) {
    throw std::runtime_error("short read from " + raw_path);
  }
  return data;
}

FastProcessPipeline::FastProcessPipeline(
  const fs::path & context_dir,
  const std::string & matrix_type,
  bool median_blur)
: median_blur_(median_blur),
  use_correction_matrix_(!matrix_type.empty()),
  matrix_type_(matrix_type)
{
  const ContextPaths ctx = resolve_context(context_dir);
  parse_calibration_bands(ctx.calibration_xml);

  float integration_ms = 0.f;
  const cv::Mat white = load_roi_frame(ctx.white_non_uniformity_xml, &integration_ms);
  const cv::Mat dark = load_roi_frame(ctx.dark_non_uniformity_xml, nullptr);
  if (white.size() != dark.size() || white.type() != CV_32FC1 || dark.type() != CV_32FC1) {
    throw std::runtime_error("white/dark non-uniformity shape/type mismatch");
  }
  get_gain_and_bias(white, dark);
  reference_panel_spectrum_.assign(wavelengths_nm_.size(), 0.95f);
}

void FastProcessPipeline::parse_calibration_bands(const fs::path & calibration_xml)
{
  tinyxml2::XMLDocument doc;
  if (doc.LoadFile(calibration_xml.string().c_str()) != tinyxml2::XML_SUCCESS) {
    throw std::runtime_error("failed to parse calibration " + calibration_xml.string());
  }
  const auto * root = doc.RootElement();
  const auto * filter_zone = find_named(root, "filter_zone");
  if (filter_zone == nullptr) {
    throw std::runtime_error("filter_zone not found in calibration xml");
  }
  const auto * pw = filter_zone->FirstChildElement("pattern_width");
  if (pw == nullptr || pw->GetText() == nullptr) {
    throw std::runtime_error("pattern_width missing");
  }
  mosaic_size_ = std::stoi(pw->GetText());

  const tinyxml2::XMLElement * corr = nullptr;
  std::function<void(const tinyxml2::XMLElement *)> find_corr =
    [&](const tinyxml2::XMLElement * node) {
      if (node == nullptr || corr != nullptr) {
        return;
      }
      if (std::strcmp(node->Name(), "correction_matrix") == 0) {
        const auto * name_el = node->FirstChildElement("name");
        if (name_el != nullptr && name_el->GetText() != nullptr &&
          matrix_type_ == name_el->GetText())
        {
          corr = node;
          return;
        }
      }
      for (const auto * c = node->FirstChildElement(); c != nullptr;
        c = c->NextSiblingElement())
      {
        find_corr(c);
      }
    };
  find_corr(root);

  wavelengths_nm_.clear();
  mosaic_indices_.clear();
  correction_matrix_T_.release();

  if (corr != nullptr && use_correction_matrix_) {
    const auto * vbs = corr->FirstChildElement("virtual_bands");
    if (vbs == nullptr) {
      throw std::runtime_error("virtual_bands missing");
    }
    struct VirtualBand
    {
      float wavelength_nm;
      std::vector<float> coefficients;
    };
    std::vector<VirtualBand> bands;
    for (const auto * vb = vbs->FirstChildElement("virtual_band"); vb != nullptr;
      vb = vb->NextSiblingElement("virtual_band"))
    {
      const auto * wl = vb->FirstChildElement("wavelength_nm");
      const auto * coef = vb->FirstChildElement("coefficients");
      if (wl == nullptr || wl->GetText() == nullptr || coef == nullptr) {
        throw std::runtime_error("bad virtual_band");
      }
      VirtualBand item;
      item.wavelength_nm = std::stof(wl->GetText());
      const char * values = coef->Attribute("values");
      if (values == nullptr) {
        throw std::runtime_error("coefficients values missing");
      }
      item.coefficients = parse_float_list(values);
      bands.push_back(std::move(item));
    }
    std::sort(bands.begin(), bands.end(), [](const VirtualBand & a, const VirtualBand & b) {
      return a.wavelength_nm < b.wavelength_nm;
    });
    if (bands.empty()) {
      throw std::runtime_error("no virtual bands");
    }
    const int n_virtual = static_cast<int>(bands.size());
    const int n_mosaic = static_cast<int>(bands.front().coefficients.size());
    cv::Mat corr_rows(n_virtual, n_mosaic, CV_32FC1);
    for (int i = 0; i < n_virtual; ++i) {
      if (static_cast<int>(bands[static_cast<size_t>(i)].coefficients.size()) != n_mosaic) {
        throw std::runtime_error("inconsistent coefficient length");
      }
      wavelengths_nm_.push_back(bands[static_cast<size_t>(i)].wavelength_nm);
      for (int j = 0; j < n_mosaic; ++j) {
        corr_rows.at<float>(i, j) =
          bands[static_cast<size_t>(i)].coefficients[static_cast<size_t>(j)];
      }
    }
    correction_matrix_T_ = corr_rows.t();
    return;
  }

  const auto * bands_el = filter_zone->FirstChildElement("bands");
  if (bands_el == nullptr) {
    throw std::runtime_error("bands missing and no correction matrix");
  }
  struct Band
  {
    int mosaic_index;
    float wavelength_nm;
  };
  std::vector<Band> bands;
  for (const auto * band = bands_el->FirstChildElement("band"); band != nullptr;
    band = band->NextSiblingElement("band"))
  {
    const char * selected = band->Attribute("selected");
    if (selected == nullptr || std::string(selected) != "true") {
      continue;
    }
    const char * index = band->Attribute("index");
    if (index == nullptr) {
      continue;
    }
    const auto * wl_el = find_named(band, "wavelength_nm");
    if (wl_el == nullptr || wl_el->GetText() == nullptr) {
      continue;
    }
    bands.push_back(Band{std::stoi(index), std::stof(wl_el->GetText())});
  }
  std::sort(bands.begin(), bands.end(), [](const Band & a, const Band & b) {
    return a.wavelength_nm < b.wavelength_nm;
  });
  for (const auto & b : bands) {
    mosaic_indices_.push_back(b.mosaic_index);
    wavelengths_nm_.push_back(b.wavelength_nm);
  }
}

void FastProcessPipeline::get_gain_and_bias(const cv::Mat & white, const cv::Mat & dark)
{
  cv::Mat denom;
  cv::subtract(white, dark, denom);

  // eps from median of positive (white-dark); mask positives with OpenCV compare.
  cv::Mat positive_mask;
  cv::compare(denom, 0.0, positive_mask, cv::CMP_GT);
  const int n_pos = cv::countNonZero(positive_mask);
  float eps = 1e-3f;
  if (n_pos > 0) {
    std::vector<float> positive;
    positive.reserve(static_cast<size_t>(n_pos));
    for (int r = 0; r < denom.rows; ++r) {
      const float * row = denom.ptr<float>(r);
      const uint8_t * m = positive_mask.ptr<uint8_t>(r);
      for (int c = 0; c < denom.cols; ++c) {
        if (m[c]) {
          positive.push_back(row[c]);
        }
      }
    }
    const size_t mid = positive.size() / 2;
    std::nth_element(
      positive.begin(), positive.begin() + static_cast<std::ptrdiff_t>(mid), positive.end());
    eps = std::max(1e-3f, 1e-6f * positive[mid]);
  }

  cv::max(denom, eps, denom);
  cv::divide(1.0, denom, gain_);
  cv::multiply(dark, gain_, bias_, -1.0);  // bias = -dark * gain
}

cv::Mat FastProcessPipeline::demosaic_all_bands(const cv::Mat & frame) const
{
  const int out_h = frame.rows / mosaic_size_;
  const int out_w = frame.cols / mosaic_size_;
  const int n_bands = mosaic_size_ * mosaic_size_;

  // Exact mosaic stride sample: frame[dy::m, dx::m] — ROI+resize overruns when dx/dy > 0.
  std::vector<cv::Mat> chans(static_cast<size_t>(n_bands));
  for (int mosaic_index = 0; mosaic_index < n_bands; ++mosaic_index) {
    const int dx = mosaic_index % mosaic_size_;
    const int dy = mosaic_index / mosaic_size_;
    cv::Mat band(out_h, out_w, CV_32FC1);
    for (int y = 0; y < out_h; ++y) {
      const float * src = frame.ptr<float>(dy + y * mosaic_size_);
      float * dst = band.ptr<float>(y);
      for (int x = 0; x < out_w; ++x) {
        dst[x] = src[dx + x * mosaic_size_];
      }
    }
    chans[static_cast<size_t>(mosaic_index)] = band;
  }
  cv::Mat multi;
  cv::merge(chans, multi);
  return multi;
}

cv::Mat FastProcessPipeline::apply_spectral_correction(const cv::Mat & cube) const
{
  const int n_mosaic = correction_matrix_T_.rows;
  const int n_virtual = correction_matrix_T_.cols;
  if (cube.channels() != n_mosaic) {
    throw std::runtime_error("cube channel count does not match mosaic size for correction");
  }
  const int out_h = cube.rows;
  CV_Assert(cube.isContinuous());

  // (H*W, n_mosaic) @ (n_mosaic, n_virtual) → (H*W, n_virtual), then reshape to multi-channel image.
  const cv::Mat flat = cube.reshape(1, out_h * cube.cols);
  cv::Mat corrected;
  cv::gemm(flat, correction_matrix_T_, 1.0, cv::Mat(), 0.0, corrected);
  CV_Assert(corrected.isContinuous());
  return corrected.reshape(n_virtual, out_h);  // (out_h, out_w, n_virtual)
}

cv::Mat FastProcessPipeline::median_filter_cube(const cv::Mat & cube) const
{
  std::vector<cv::Mat> chans;
  cv::split(cube, chans);
  for (auto & ch : chans) {
    cv::medianBlur(ch, ch, 3);
  }
  cv::Mat out;
  cv::merge(chans, out);
  return out;
}

cv::Mat FastProcessPipeline::fast_process_to_reflectance(const cv::Mat & raw_scene_frame) const
{
  if (raw_scene_frame.type() != CV_32FC1) {
    throw std::runtime_error("raw frame must be CV_32FC1");
  }
  if (raw_scene_frame.size() != gain_.size()) {
    throw std::runtime_error(
      "raw frame size " + std::to_string(raw_scene_frame.cols) + "x" +
      std::to_string(raw_scene_frame.rows) + " != gain " +
      std::to_string(gain_.cols) + "x" + std::to_string(gain_.rows));
  }

  cv::Mat reflectance_frame;
  cv::multiply(raw_scene_frame, gain_, reflectance_frame);
  cv::add(reflectance_frame, bias_, reflectance_frame);

  cv::Mat cube;
  if (!correction_matrix_T_.empty()) {
    cube = apply_spectral_correction(demosaic_all_bands(reflectance_frame));
  } else {
    const int out_h = reflectance_frame.rows / mosaic_size_;
    const int out_w = reflectance_frame.cols / mosaic_size_;
    const int n_bands = static_cast<int>(mosaic_indices_.size());
    std::vector<cv::Mat> chans(static_cast<size_t>(n_bands));
    for (int band_i = 0; band_i < n_bands; ++band_i) {
      const int mosaic_index = mosaic_indices_[static_cast<size_t>(band_i)];
      const int dx = mosaic_index % mosaic_size_;
      const int dy = mosaic_index / mosaic_size_;
      cv::Mat band(out_h, out_w, CV_32FC1);
      for (int y = 0; y < out_h; ++y) {
        const float * src = reflectance_frame.ptr<float>(dy + y * mosaic_size_);
        float * dst = band.ptr<float>(y);
        for (int x = 0; x < out_w; ++x) {
          dst[x] = src[dx + x * mosaic_size_];
        }
      }
      chans[static_cast<size_t>(band_i)] = band;
    }
    cv::merge(chans, cube);
  }

  const int bands = cube.channels();
  if (static_cast<int>(reference_panel_spectrum_.size()) != bands) {
    throw std::runtime_error("reference spectrum length mismatch");
  }
  std::vector<cv::Mat> chans;
  cv::split(cube, chans);
  for (int b = 0; b < bands; ++b) {
    chans[static_cast<size_t>(b)] *= reference_panel_spectrum_[static_cast<size_t>(b)];
  }
  cv::merge(chans, cube);

  if (median_blur_) {
    cube = median_filter_cube(cube);
  }
  return cube;
}

}  // namespace radiometric_processing_cpp
