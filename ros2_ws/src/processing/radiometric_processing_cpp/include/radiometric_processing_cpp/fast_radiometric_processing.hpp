#pragma once

#include <opencv2/core.hpp>

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace radiometric_processing_cpp
{

struct ContextPaths
{
  std::filesystem::path calibration_xml;
  std::filesystem::path white_non_uniformity_xml;
  std::filesystem::path dark_non_uniformity_xml;
  std::filesystem::path white_reference_xml;
  std::vector<std::filesystem::path> dark_reference_xmls;
};

ContextPaths resolve_context(const std::filesystem::path & context_dir);

/** Online-equivalent of Python FastProcessPipeline (gain/bias + demosaic + spectral matrix). */
class FastProcessPipeline
{
public:
  FastProcessPipeline(
    const std::filesystem::path & context_dir,
    const std::string & matrix_type = "hsi_irradiance",
    bool median_blur = false);

  /** raw_scene_frame: CV_32FC1, same geometry as white/dark references. */
  cv::Mat fast_process_to_reflectance(const cv::Mat & raw_scene_frame) const;

  const std::vector<float> & wavelengths_nm() const { return wavelengths_nm_; }
  int mosaic_size() const { return mosaic_size_; }

private:
  void parse_calibration_bands(const std::filesystem::path & calibration_xml);
  void get_gain_and_bias(const cv::Mat & white, const cv::Mat & dark);
  cv::Mat demosaic_all_bands(const cv::Mat & frame) const;
  cv::Mat apply_spectral_correction(const cv::Mat & cube) const;
  cv::Mat median_filter_cube(const cv::Mat & cube) const;

  bool median_blur_{false};
  bool use_correction_matrix_{false};
  std::string matrix_type_;

  int mosaic_size_{0};
  std::vector<int> mosaic_indices_;
  std::vector<float> wavelengths_nm_;
  /** Rows = mosaic bands, cols = virtual bands (Python correction_matrix.T). */
  cv::Mat correction_matrix_T_;
  cv::Mat gain_;
  cv::Mat bias_;
  /** Per-band panel scale; applied via split / scalar multiply / merge. */
  std::vector<float> reference_panel_spectrum_;
};

cv::Mat load_roi_frame(const std::filesystem::path & xml_path, float * integration_time_ms = nullptr);

}  // namespace radiometric_processing_cpp
