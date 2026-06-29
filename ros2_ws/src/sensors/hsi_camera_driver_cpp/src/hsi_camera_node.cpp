#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/qos.hpp>
#include "hsi_camera_driver_cpp/srv/camera_commands.hpp"
#include <xiApi.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cctype>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

extern "C" {
#include "hsi_api_common.h"
#include "hsi_camera_api.h"
}

namespace {
constexpr int kMaxCameraInfos = 8;
constexpr const char * kVisModel = "MQ022HG-IM-SM4X4-VIS3";
constexpr const char * kNirModel = "MQ022HG-IM-SM5X5-NIR2";
constexpr const char * kDataRoot = "/mnt/data";
constexpr const char * kTemplateFilesSubdir = "template_files";
constexpr const char * kVisCalibrationXml = "CMV2K-SSM4x4-460_600-15.7.18.12.xml";
constexpr const char * kNirCalibrationXml = "CMV2K-SSM5x5-665_975-18.2.11.9.xml";
constexpr const char * kContextDescXml = "context_description.xml";
constexpr const char * kOpticalSetupXml = "optical_setup.xml";
// Active sensor areas from calibration files (offset_x/y are zero for both).
constexpr int kVisActiveWidth = 2048;
constexpr int kVisActiveHeight = 1088;
constexpr int kNirActiveWidth = 2045;
constexpr int kNirActiveHeight = 1085;


std::string field_as_string(const char * field) {
  std::string s(field);
  s.erase(std::find(s.begin(), s.end(), '\0'), s.end());
  return s;
}

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
}  // namespace

class HsiCameraAcquisitionNode : public rclcpp::Node {
public:
  enum class CommandType
  {
    kIdle,
    kAutoExposure, 
    kWhiteReferenceFull,
    kWhiteReferenceReference, 
    kDarkReference, 
    kTimelapse,
    kManual, 
    kTestFrame 
  };

  HsiCameraAcquisitionNode() : Node("hsi_camera_acquisition_node") {
    declare_parameter<bool>("free_run", true);
    declare_parameter<double>("acquisition_rate_hz", 20.0);
    declare_parameter<double>("throttled_rate_hz", 1.0);
    declare_parameter<std::string>("device_name", "");
    declare_parameter<double>("exposure_time_ms", 10.0);
    declare_parameter<double>("gain", 1.0);
    declare_parameter<int>("test_frame_count", -1);
    declare_parameter<int>("camera_start_delay_ms", 0);
    declare_parameter<std::string>("mission_state_topic", "/mission_state");
    declare_parameter<int>("raw_mono_value_max", 1023);
    declare_parameter<std::string>("run_name_path", "");
    

    free_run_ = get_parameter("free_run").as_bool();
    acquisition_rate_hz_ = get_parameter("acquisition_rate_hz").as_double();
    throttled_rate_hz_ = get_parameter("throttled_rate_hz").as_double();
    device_name_ = get_parameter("device_name").as_string();
    exposure_time_ms_ = get_parameter("exposure_time_ms").as_double();
    gain_ = get_parameter("gain").as_double();
    test_frame_count_ = get_parameter("test_frame_count").as_int();
    camera_start_delay_ms_ = get_parameter("camera_start_delay_ms").as_int();
    mission_state_topic_ = get_parameter("mission_state_topic").as_string();
    run_name_path_ = get_parameter("run_name_path").as_string();
    if (run_name_path_.empty()) {
      throw std::runtime_error("run_name_path parameter must be set to an absolute output directory");
    }
    raw_mono_value_max_ = get_parameter("raw_mono_value_max").as_int();
    if (raw_mono_value_max_ < 1) {
      raw_mono_value_max_ = 65535;
    }

    
    if (exposure_time_ms_ > 0.0 && exposure_time_ms_ * acquisition_rate_hz_ > 990.0) {
      acquisition_rate_hz_ = 990.0 / exposure_time_ms_;
      RCLCPP_WARN(
        get_logger(),
        "Requested exposure*rate exceeds SDK limit; clamping acquisition_rate_hz to %.3f",
        acquisition_rate_hz_);
      }
      
      if (throttled_rate_hz_ <= 0.0) {
      RCLCPP_ERROR(get_logger(), "throttled_rate_hz must be > 0");
    }
    
    RCLCPP_INFO(
      get_logger(),
      "Parameters: free_run=%s acquisition_rate_hz=%.6f throttled_rate_hz=%.6f device_name='%s' "
      "exposure_time_ms=%.6f gain=%.6f "
      "test_frame_count=%d camera_start_delay_ms=%d mission_state_topic='%s' raw_mono_value_max=%d",
      free_run_ ? "true" : "false",
      acquisition_rate_hz_,
      throttled_rate_hz_,
      device_name_.c_str(),
      exposure_time_ms_,
      gain_,
      test_frame_count_,
      camera_start_delay_ms_,
      mission_state_topic_.c_str(),
      raw_mono_value_max_);
      
      const auto topic_base = std::string("/hsi/") + device_name_;
      
      rclcpp::QoS qos(rclcpp::KeepLast(1));
      qos.best_effort();
      raw_publisher_ = create_publisher<sensor_msgs::msg::Image>(topic_base + "/raw", qos);
      raw_throttled_publisher_ =
        create_publisher<sensor_msgs::msg::Image>(topic_base + "/raw/throttled", qos);
      temp_publisher_ = create_publisher<std_msgs::msg::Float32>(topic_base + "/temp", qos);
      run_dir_publisher_ = create_publisher<std_msgs::msg::String>("/run_save_dir", qos);
      exposure_actual_publisher_ = create_publisher<std_msgs::msg::Float32>(topic_base + "/exposure_time_actual", rclcpp::QoS(10));
      
      const auto run_output_dir = setup_run_output_dir(run_name_path_);
      RCLCPP_INFO(get_logger(), "Run output dir: %s", run_output_dir.c_str());

      exposure_setpoint_sub_ = create_subscription<std_msgs::msg::Float32>(
        topic_base + "/exposure_time_setpoint", rclcpp::QoS(10),
        [this](const std_msgs::msg::Float32 & msg) { request_exposure_update(msg.data); });
        
        mission_sub_ = create_subscription<std_msgs::msg::String>(
          mission_state_topic_, rclcpp::QoS(10),
      [this](const std_msgs::msg::String & msg) {
        if (to_lower_copy(trim_copy(msg.data)) == "shutdown") {
          RCLCPP_INFO(get_logger(), "mission_state shutdown - exiting cleanly");
          rclcpp::shutdown();
        }
      });
    
    const auto timelapse_period = std::chrono::duration<double>(1.0 / throttled_rate_hz_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(timelapse_period),
      std::bind(&HsiCameraAcquisitionNode::timelapse_metronome, this));

    camera_commands_srv_ = create_service<hsi_camera_driver_cpp::srv::CameraCommands>(
      topic_base + "/camera_commands",
      [this](
        const std::shared_ptr<hsi_camera_driver_cpp::srv::CameraCommands::Request> req,
        std::shared_ptr<hsi_camera_driver_cpp::srv::CameraCommands::Response> resp) {
        CommandType parsed = CommandType::kIdle;
        if (!try_parse_command_type(req->command, parsed)) {
          resp->success = false;
          resp->message =
            "invalid mode; use one of: white_reference_full/white_reference_reference/"
            "dark_reference/auto_exposure/timelapse/idle/manual/test_frame";
          RCLCPP_WARN(
            get_logger(),
            "Rejected Camera Command '%s': expected white_reference_full|white_reference_reference|"
            "dark_reference|auto_exposure|timelapse|idle|manual|test_frame",
            req->command.c_str());
          return;
        }

        {
          std::lock_guard<std::mutex> lock(mutex_);
          camera_command_request_ = parsed;
        }
        {
          std::lock_guard<std::mutex> lock(command_state_mutex_);
          command_requested_ = true;
          command_completed_ = false;
          command_ok_ = false;
          command_status_message_.clear();
        }

        std::unique_lock<std::mutex> lock(command_state_mutex_);
        const bool completed = command_state_cv_.wait_for(
          lock, std::chrono::seconds(2), [this]() { return command_completed_; });
        if (!completed) {
          resp->success = false;
          resp->message = "command timed out waiting for frame save";
          return;
        }

        resp->success = command_ok_;
        resp->message = command_ok_
          ? ("command applied and frame saved for " + command_type_to_string(parsed))
          : ("command failed: " + command_status_message_);
        RCLCPP_INFO(
          get_logger(),
          "Camera command '%s' completed, success=%s",
          command_type_to_string(parsed).c_str(),
          command_ok_ ? "true" : "false");
      });

    initialize_sdk_logger();
    log_api_versions();
    init_camera();
    publish_exposure_time_actual();

    worker_thread_ = std::thread(&HsiCameraAcquisitionNode::acquisition_loop, this);
  }

  ~HsiCameraAcquisitionNode() override {
    stop_requested_.store(true);
    if (worker_thread_.joinable()) {
      worker_thread_.join();
    }
    cleanup_camera();
  }

private:
  void timelapse_metronome(){
    if (!timelapse_state_) {
      timelapse_tick_ = false;
    } else {
      timelapse_tick_ = true;
    }
  }

  bool try_parse_command_type(const std::string & s, CommandType & out) const
  {
    const std::string t = to_lower_copy(trim_copy(s));
    if (t == "idle") {
      out = CommandType::kIdle;
      return true;
    }
    if (t == "auto_exposure") {
      out = CommandType::kAutoExposure;
      return true;
    }
    if (t == "white_reference_full") {
      out = CommandType::kWhiteReferenceFull;
      return true;
    }
    if (t == "white_reference_reference") {
      out = CommandType::kWhiteReferenceReference;
      return true;
    }
    if (t == "dark_reference") {
      out = CommandType::kDarkReference;
      return true;
    }
    if (t == "timelapse") {
      out = CommandType::kTimelapse;
      return true;
    }
    if (t == "manual") {
      out = CommandType::kManual;
      return true;
    }
    if (t == "test_frame") {
      out = CommandType::kTestFrame;
      return true;
    }
    return false;
  }

  CommandType normalize_command_type(const std::string & s) const
  {
    CommandType parsed = CommandType::kIdle;
    if (!try_parse_command_type(s, parsed)) {
      RCLCPP_WARN(
        get_logger(),
        "command_type '%s' is invalid",
        s.c_str());
      return CommandType::kIdle;
    }
    return parsed;
  }

  std::string command_type_to_string(CommandType t) const
  {
    if (t == CommandType::kIdle) {
      return "idle";
    }
    if (t == CommandType::kAutoExposure) {
      return "auto_exposure";
    }
    if (t == CommandType::kWhiteReferenceFull) {
      return "white_reference_full";
    }
    if (t == CommandType::kWhiteReferenceReference) {
      return "white_reference_reference";
    }
    if (t == CommandType::kDarkReference) {
      return "dark_reference";
    }
    if (t == CommandType::kTimelapse) {
      return "time_lapse";
    }
    if (t == CommandType::kManual) {
      return "manual";
    }
    if (t == CommandType::kTestFrame ) {
      return "test_frame";
    }

    return "idle";
  }

  std::filesystem::path setup_run_output_dir(const std::string & run_dir) {
    const auto dir = std::filesystem::path(run_dir);
    if (run_dir_publisher_) {
      std_msgs::msg::String msg;
      msg.data = dir.string();
      run_dir_publisher_->publish(msg);
    }
    std::filesystem::create_directories(dir);

    camera_dir_ = dir / device_name_.c_str();
    raw_dir_ = camera_dir_ / "raw";
    context_dir_ = raw_dir_ / "context";
    dark_dir_ = context_dir_ / "dark_references";
    white_dir_ = context_dir_ / "white_reference";
    non_uniformity_dir_ = context_dir_ / "non_uniformity";
    calibration_dir_ = context_dir_ / "calibration_file";
    test_image_dir_ = context_dir_ / "test_image";
    optical_setup_dir_ = context_dir_ / "optical_setup";

    std::filesystem::create_directories(camera_dir_);
    std::filesystem::create_directories(raw_dir_);
    std::filesystem::create_directories(context_dir_);
    std::filesystem::create_directories(dark_dir_);
    std::filesystem::create_directories(white_dir_);
    std::filesystem::create_directories(non_uniformity_dir_);
    std::filesystem::create_directories(calibration_dir_);
    std::filesystem::create_directories(test_image_dir_);
    std::filesystem::create_directories(optical_setup_dir_);

    const std::string device_lower = to_lower_copy(trim_copy(device_name_));
    const char * calibration_xml =
      (device_lower == "nir") ? kNirCalibrationXml : kVisCalibrationXml;
    const auto template_dir =
      std::filesystem::path(kDataRoot) / kTemplateFilesSubdir / device_lower;

    const std::array<std::pair<std::filesystem::path, std::filesystem::path>, 3> template_copies{{
      {template_dir / calibration_xml, calibration_dir_ / calibration_xml},
      {template_dir / kOpticalSetupXml, optical_setup_dir_ / kOpticalSetupXml},
      {template_dir / kContextDescXml, context_dir_ / kContextDescXml},
    }};

    std::array<std::error_code, template_copies.size()> copy_errors{};
    for (std::size_t i = 0; i < template_copies.size(); ++i) {
      std::filesystem::copy_file(
        template_copies[i].first, template_copies[i].second,
        std::filesystem::copy_options::overwrite_existing, copy_errors[i]);
    }

    for (std::size_t i = 0; i < template_copies.size(); ++i) {
      const auto & src = template_copies[i].first;
      const auto & dst = template_copies[i].second;
      if (copy_errors[i]) {
        RCLCPP_WARN(
          get_logger(), "Failed to copy context template file '%s' -> '%s': %s",
          src.c_str(), dst.c_str(), copy_errors[i].message().c_str());
      } else {
        RCLCPP_INFO(
          get_logger(), "Copied context template file '%s' -> '%s'",
          src.c_str(), dst.c_str());
      }
    }

    return dir;
  }

  void initialize_sdk_logger() {
    const HSI_RETURN err = commonInitializeLogger("./logs", LV_VERBOSE);
    if (err != HSI_OK) {
      RCLCPP_WARN(get_logger(), "commonInitializeLogger failed with code %d", err);
    }
  }

  void log_api_versions() {
    int major = 0;
    int minor = 0;
    int patch = 0;
    int build = 0;

    if (commonGetAPIVersion(&major, &minor, &patch, &build) == HSI_OK) {
      RCLCPP_INFO(get_logger(), "COMMON API: %d.%d.%d.%d", major, minor, patch, build);
    }
    if (cameraGetAPIVersion(&major, &minor, &patch, &build) == HSI_OK) {
      RCLCPP_INFO(get_logger(), "CAMERA API: %d.%d.%d.%d", major, minor, patch, build);
    }
  }

  void init_camera() {
    std::vector<CameraInfo> infos(kMaxCameraInfos);
    int found = 0;
    DeviceArgs args{};
    args.manufacturer = EM_ALL;
    args.model = MO_ALL;

    if (camera_start_delay_ms_ > 0) {
      RCLCPP_INFO(get_logger(), "Waiting %d ms before cameraStart() ...", camera_start_delay_ms_);
      std::this_thread::sleep_for(std::chrono::milliseconds(camera_start_delay_ms_));
    }

    check(
      cameraEnumerateConnectedDevices(infos.data(), &found, static_cast<int>(infos.size()), args),
      "cameraEnumerateConnectedDevices");

    if (found <= 0) {
      throw std::runtime_error("No cameras found");
    }

    RCLCPP_INFO(get_logger(), "Enumerated %d camera(s)", found);
    for (int i = 0; i < found; ++i) {
      const CameraInfo & cam = infos[static_cast<size_t>(i)];
      const std::string model = field_as_string(cam.model);
      const std::string serial = field_as_string(cam.serial_number);
      const std::string id_str = field_as_string(cam.identification_string);
      RCLCPP_INFO(
        get_logger(), "  [%d] model='%s' serial='%s' id='%s'", i, model.c_str(), serial.c_str(),
        id_str.c_str());
    }

    const int selected_index = select_camera_index(infos, found);
    RCLCPP_INFO(get_logger(), "Running camera %d", selected_index);

    check(cameraOpenDevice(&camera_, infos.at(static_cast<size_t>(selected_index))), "cameraOpenDevice");
    check_xiapi(xiOpenDevice(selected_index, &camera_xiapi_), "xiOpenDevice");
    apply_camera_active_area_roi("init_camera");
    check(cameraInitialize(camera_), "cameraInitialize");
    check(cameraGetOutputFrameDataFormat(camera_, &frame_format_), "cameraGetOutputFrameDataFormat");
    check(commonAllocateFrame(&frame_, frame_format_), "commonAllocateFrame");

    cameraRuntimeParameters runtime{};
    check(cameraGetRuntimeParameters(camera_, &runtime), "cameraGetRuntimeParameters");
    runtime.exposure_time_ms = exposure_time_ms_;
    runtime.gain = gain_;
    runtime.frame_rate_hz = acquisition_rate_hz_;
    runtime.trigger_mode = free_run_ ? TM_NoTriggering : TM_SoftwareTriggered;
    check(cameraSetRuntimeParameters(camera_, runtime), "cameraSetRuntimeParameters");

    check(cameraStart(camera_), "cameraStart");
    camera_started_ = true;
  }

  int select_camera_index(const std::vector<CameraInfo> & infos, int found) const {
    std::string name_lower = device_name_;
    for (auto & c : name_lower) {
      c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }

    if (name_lower != "vis" && name_lower != "nir") {
      return 0;
    }

    const std::string wanted_model = (name_lower == "vis") ? kVisModel : kNirModel;
    for (int i = 0; i < found; ++i) {
      if (wanted_model == field_as_string(infos.at(static_cast<size_t>(i)).model)) {
        return i;
      }
    }

    throw std::runtime_error(
      "Requested device_name '" + name_lower + "' model '" + wanted_model + "' was not found");
  }

  void acquisition_loop() {
    const auto sleep_period =
      std::chrono::duration<double>(acquisition_rate_hz_ > 0.0 ? (1.0 / acquisition_rate_hz_) : 0.0);
    int frame_count = 0;
    int saved_frame_count = 0;
    bool command_success = false;
    int overexposed_pixel_count = 1;
    double step = 0.0;
    double new_exposure = 0.0;
    float temp = 0.0;
    std::string command_success_message = "";
    std::string command_failed_message = "";

    const auto start_time = std::chrono::steady_clock::now();
    while (rclcpp::ok() && !stop_requested_.load()) {
      apply_pending_exposure_update();
      const HSI_RETURN acquire_err = cameraAcquireFrame(camera_, &frame_, 0);
      if (acquire_err != HSI_OK) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "cameraAcquireFrame failed with code %d", acquire_err);
        continue;
      }

      publish_frame();

      // get sensor temp
      const XI_RETURN temp_err = xiGetParamFloat(camera_xiapi_, XI_PRM_SENSOR_BOARD_TEMP, &temp);
      if (temp_err != XI_OK) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, 
          "xiGetParamFloat(camera_, XI_PRM_SENSOR_BOARD_TEMP, &temp) %d", temp_err);
      } else {
        // RCLCPP_INFO(get_logger(), "Sensor Board Temp: %.2f\n", temp);
        std_msgs::msg::Float32 temp_msg;
        temp_msg.data = temp;
        temp_publisher_->publish(temp_msg);
      }

      // record timelapse if in timelapse mode
      if (timelapse_tick_) {
        const std::string command_dir_str = raw_dir_.string();
        const std::string frame_name = "timelapse_" + std::to_string(saved_frame_count);
        const HSI_RETURN tl_save_err =
          commonSaveFrame(frame_, command_dir_str.c_str(), frame_name.c_str(), FF_RAW);
        raw_throttled_publisher_->publish(build_image_msg());
        timelapse_tick_ = false;
        if (tl_save_err == HSI_OK) {
          ++saved_frame_count;
        }
      }

      bool command_requested_copy = false;
      CommandType camera_command_request_copy = CommandType::kIdle;
      {
        std::lock_guard<std::mutex> lock(command_state_mutex_);
        command_requested_copy = command_requested_;
      }
      if (command_requested_copy) {
        HSI_RETURN save_err = HSI_OK;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          camera_command_request_copy = camera_command_request_;
        }
        switch (camera_command_request_copy) {
          case CommandType::kWhiteReferenceFull: {
            // record one of many white reference full coverage
            const std::string command_dir_str = non_uniformity_dir_.string();
            const std::string frame_name = "white_reference_" +  std::to_string(saved_frame_count);
            save_err = commonSaveFrame(frame_, command_dir_str.c_str(), frame_name.c_str(), FF_RAW);
            command_success = (save_err == HSI_OK);
            command_success_message = ("saved frame " + frame_name + " for " + command_type_to_string(camera_command_request_copy));
            command_failed_message = ("commonSaveFrame failed with code " + std::to_string(save_err));
            break;
          }
          case CommandType::kWhiteReferenceReference: {
            // record one of many white reference not full coverage
            const std::string command_dir_str = white_dir_.string();
            const std::string frame_name = "white_reference_" +  std::to_string(saved_frame_count);
            save_err = commonSaveFrame(frame_, command_dir_str.c_str(), frame_name.c_str(), FF_RAW);
            command_success = (save_err == HSI_OK);
            command_success_message = ("saved frame " + frame_name + " for " + command_type_to_string(camera_command_request_copy));
            command_failed_message = ("commonSaveFrame failed with code " + std::to_string(save_err));
            break;
          }
          case CommandType::kDarkReference: {
            //record one of multiple dark references, including exp time
            const std::string command_dir_str = dark_dir_.string();
            const std::string frame_name = "dark_reference_" +  std::to_string(saved_frame_count) + "_" + std::to_string(std::round(exposure_time_ms_));
            save_err = commonSaveFrame(frame_, command_dir_str.c_str(), frame_name.c_str(), FF_RAW);
            command_success = (save_err == HSI_OK);
            command_success_message = ("saved frame " + frame_name + " for " + command_type_to_string(camera_command_request_copy));
            command_failed_message = ("commonSaveFrame failed with code " + std::to_string(save_err));
            break;
          }
          case CommandType::kTestFrame: {
            // record a test frame for later usage e.g to use its metadata
            const std::string command_dir_str = test_image_dir_.string();
            const std::string frame_name = "test_frame_" + std::to_string(saved_frame_count);
            save_err = commonSaveFrame(frame_, command_dir_str.c_str(), frame_name.c_str(), FF_RAW);
            command_success = (save_err == HSI_OK);
            command_success_message = ("saved frame " + frame_name + " for " + command_type_to_string(camera_command_request_copy));
            command_failed_message = ("commonSaveFrame failed with code " + std::to_string(save_err));
            break;
          }
          case CommandType::kManual: {
            // record a frame in main dir
            const std::string command_dir_str = raw_dir_.string();
            const std::string frame_name = "manual_" + std::to_string(saved_frame_count);
            save_err = commonSaveFrame(frame_, command_dir_str.c_str(), frame_name.c_str(), FF_RAW);
            command_success = (save_err == HSI_OK);
            command_success_message = ("saved frame " + frame_name + " for " + command_type_to_string(camera_command_request_copy));
            command_failed_message = ("commonSaveFrame failed with code " + std::to_string(save_err));
            break;
          }
          case CommandType::kTimelapse: {
            // record a frame in main dir, first frame will be out of sync with rest
            const std::string command_dir_str = raw_dir_.string();
            const std::string frame_name = "timelapse_" + std::to_string(saved_frame_count);
            save_err = commonSaveFrame(frame_, command_dir_str.c_str(), frame_name.c_str(), FF_RAW);
            timelapse_state_ = true;
            command_success = true;
            command_success_message = ("Timelapse state initiated for" + command_type_to_string(camera_command_request_copy));
            command_failed_message = ("Timelapse state failed");
            break;
          }
          case CommandType::kAutoExposure:
            // record a frame in main dir, first frame will be out of sync with rest
            auto_exposure_running_ = true;
            command_success = true;
            command_success_message = ("Autoexposure state initiated for" + command_type_to_string(camera_command_request_copy));
            command_failed_message = ("Autoexposure state failed");
            break;
          case CommandType::kIdle:
            // possibly end of timelapse
            timelapse_state_ = false;
            auto_exposure_running_ = false;
            command_success = true;
            command_success_message = ("Idle state initiated for" + command_type_to_string(camera_command_request_copy));
            command_failed_message = ("Idle state failed");
            break;
          default:
            // unknown state
            command_success = false;
            command_failed_message = ("Idle state failed for " + command_type_to_string(camera_command_request_copy));
            break;
        }

        {
          std::lock_guard<std::mutex> lock(command_state_mutex_);
          command_requested_ = false;
          command_completed_ = true;
          command_ok_ = command_success;
          command_status_message_ = command_ok_
            ? command_success_message : command_failed_message;
        }
        command_state_cv_.notify_one();

        if (save_err != HSI_OK) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000, "commonSaveFrame failed with code %d", save_err);
        }
        else{
          ++saved_frame_count;
        }
      }

      if (auto_exposure_running_) {
        // simple method brings exposure down starting from an overexposed point
        overexposed_pixel_count = 0;
        for (int row = 0; row < frame_format_.height; ++row) {
          for (int col = 0; col < frame_format_.width; ++col) {
            if (static_cast<int>(frame_.pp_data[row][col]) >= 1023) {
              ++overexposed_pixel_count;
            }
          }
        }
        // increase or decrease by 1 based on overexposure
        if (overexposed_pixel_count > 0) {
          step = -1.0;
          new_exposure = exposure_time_ms_ + step;
          request_exposure_update(new_exposure);
          publish_exposure_time_actual();
        }
        else {
          auto_exposure_running_ = false;
        }
      }

      ++frame_count;
      // auto shutdown initiated if system tested with limited number of frames set
      if (test_frame_count_ >= 0 && frame_count >= test_frame_count_) {
        const auto end_time = std::chrono::steady_clock::now();
        const std::chrono::duration<double> elapsed = end_time - start_time;
        const double elapsed_seconds = elapsed.count();
        const double fps = (elapsed_seconds > 0.0)
          ? (static_cast<double>(frame_count) / elapsed_seconds)
          : 0.0;
        RCLCPP_INFO(
          get_logger(),
          "Frame-count test complete: frames=%d, elapsed=%.6f s, fps=%.3f",
          frame_count,
          elapsed_seconds,
          fps);
        rclcpp::shutdown();
        break;
      }

      if (!free_run_ && acquisition_rate_hz_ > 0.0) {
        std::this_thread::sleep_for(sleep_period);
      }
    }
  }

  sensor_msgs::msg::Image build_image_msg() {
    auto image_msg = sensor_msgs::msg::Image();
    image_msg.header.stamp = now();
    image_msg.header.frame_id = "hsi_frame";
    image_msg.height = static_cast<uint32_t>(frame_format_.height);
    image_msg.width = static_cast<uint32_t>(frame_format_.width);
    image_msg.encoding = "mono16";
    image_msg.is_bigendian = 0;
    image_msg.step =
      static_cast<sensor_msgs::msg::Image::_step_type>(frame_format_.width * sizeof(uint16_t));
    image_msg.data.resize(image_msg.step * image_msg.height);

    const float cap = static_cast<float>(raw_mono_value_max_);
    size_t byte_index = 0;
    for (int row = 0; row < frame_format_.height; ++row) {
      for (int col = 0; col < frame_format_.width; ++col) {
        float sample = frame_.pp_data[row][col];
        if (sample < 0.0f) {
          sample = 0.0f;
        }
        if (sample > cap) {
          sample = cap;
        }
        const auto pixel = static_cast<uint16_t>(sample);
        std::memcpy(image_msg.data.data() + byte_index, &pixel, sizeof(uint16_t));
        byte_index += sizeof(uint16_t);
      }
    }

    return image_msg;
  }

  void publish_frame() {
    raw_publisher_->publish(build_image_msg());
  }

  void cleanup_camera() {
    if (camera_started_) {
      (void)cameraStop(camera_);
      camera_started_ = false;
    }
    if (frame_.pp_data != nullptr) {
      (void)commonDeallocateFrame(&frame_);
    }
    if (camera_ != nullptr) {
      (void)cameraCloseDevice(&camera_);
    }
    if (camera_xiapi_ != nullptr) {
      (void)xiCloseDevice(camera_xiapi_);
      camera_xiapi_ = nullptr;
    }
  }

  void check(HSI_RETURN code, const std::string & call_name) const {
    if (code != HSI_OK) {
      throw std::runtime_error(call_name + " failed with code " + std::to_string(code));
    }
  }

  void check_xiapi(XI_RETURN code, const std::string & call_name) const {
    if (code != XI_OK) {
      throw std::runtime_error(call_name + " failed with code " + std::to_string(code));
    }
  }

  static const char * hsi_return_name(HSI_RETURN code) {
    switch (code) {
      case HSI_OK:
        return "HSI_OK";
      case HSI_CALL_ILLEGAL:
        return "HSI_CALL_ILLEGAL";
      case HSI_HANDLE_INVALID:
        return "HSI_HANDLE_INVALID";
      case HSI_ARGUMENT_INVALID:
        return "HSI_ARGUMENT_INVALID";
      default:
        return "unknown";
    }
  }

  void publish_exposure_time_actual() {
    std_msgs::msg::Float32 msg;
    msg.data = static_cast<float>(exposure_time_ms_);
    exposure_actual_publisher_->publish(msg);
  }

  void request_exposure_update(double requested_ms) {
    if (requested_ms <= 0.0) {
      RCLCPP_WARN(get_logger(), "Ignoring exposure_time_setpoint=%.6f ms (must be > 0)", requested_ms);
      return;
    }
    if (acquisition_rate_hz_ > 0.0 && requested_ms * acquisition_rate_hz_ > 990.0) {
      RCLCPP_WARN(
        get_logger(),
        "Ignoring exposure_time_setpoint=%.6f ms; exposure*acquisition_rate_hz exceeds SDK limit (%.3f*%.3f > 990)",
        requested_ms, requested_ms, acquisition_rate_hz_);
      return;
    }

    {
      std::lock_guard<std::mutex> lock(exposure_update_mutex_);
      pending_exposure_time_ms_ = requested_ms;
    }
    exposure_update_pending_.store(true);
  }

  RegionOfInterest active_area_roi_for_device() const
  {
    std::string name_lower = device_name_;
    for (auto & c : name_lower) {
      c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }

    if (name_lower == "nir") {
      return RegionOfInterest{0, 0, kNirActiveWidth, kNirActiveHeight};
    }

    return RegionOfInterest{0, 0, kVisActiveWidth, kVisActiveHeight};
  }

  void apply_camera_active_area_roi(const char * context)
  {
    RegionOfInterest roi = active_area_roi_for_device();
    const HSI_RETURN err = cameraSetRegionOfInterestArray(camera_, &roi, 1);
    if (err != HSI_OK) {
      RCLCPP_ERROR(
        get_logger(),
        "%s: cameraSetRegionOfInterestArray failed (%d, %s) for ROI %dx%d@%d,%d",
        context, err, hsi_return_name(err), roi.width, roi.height, roi.x, roi.y);
      throw std::runtime_error("cameraSetRegionOfInterestArray failed");
    }

    RCLCPP_INFO(
      get_logger(),
      "%s: set camera active area ROI x=%d y=%d width=%d height=%d (device_name='%s')",
      context, roi.x, roi.y, roi.width, roi.height, device_name_.c_str());
  }

  bool full_reinit_from_idle(const cameraRuntimeParameters & runtime, const char * context) {
    if (frame_.pp_data != nullptr) {
      (void)commonDeallocateFrame(&frame_);
    }
    frame_ = FrameFloat{};

    try {
      apply_camera_active_area_roi(context);
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "%s: %s", context, ex.what());
      return false;
    }

    HSI_RETURN err = cameraInitialize(camera_);
    if (err != HSI_OK) {
      RCLCPP_ERROR(
        get_logger(), "%s: cameraInitialize failed (%d, %s)", context, err, hsi_return_name(err));
      return false;
    }

    err = cameraGetOutputFrameDataFormat(camera_, &frame_format_);
    if (err != HSI_OK) {
      RCLCPP_ERROR(
        get_logger(), "%s: cameraGetOutputFrameDataFormat failed (%d, %s)", context, err,
        hsi_return_name(err));
      return false;
    }

    err = commonAllocateFrame(&frame_, frame_format_);
    if (err != HSI_OK) {
      RCLCPP_ERROR(
        get_logger(), "%s: commonAllocateFrame failed (%d, %s)", context, err, hsi_return_name(err));
      return false;
    }

    err = cameraSetRuntimeParameters(camera_, runtime);
    if (err != HSI_OK) {
      RCLCPP_ERROR(
        get_logger(), "%s: cameraSetRuntimeParameters failed (%d, %s)", context, err,
        hsi_return_name(err));
      return false;
    }

    if (!camera_start_with_retry(context)) {
      return false;
    }
    return true;
  }

  bool camera_start_with_retry(const char * context) {
    constexpr int kMaxStartAttempts = 5;
    for (int attempt = 1; attempt <= kMaxStartAttempts; ++attempt) {
      const HSI_RETURN start_err = cameraStart(camera_);
      if (start_err == HSI_OK) {
        camera_started_ = true;
        return true;
      }
      RCLCPP_ERROR(
        get_logger(),
        "%s: cameraStart attempt %d/%d failed (%d, %s) - need READY state before Start",
        context,
        attempt,
        kMaxStartAttempts,
        start_err,
        hsi_return_name(start_err));
      std::this_thread::sleep_for(std::chrono::milliseconds(150));
    }
    return false;
  }

  void apply_pending_exposure_update() {
    if (!exposure_update_pending_.exchange(false)) {
      return;
    }

    double requested_ms = 0.0;
    {
      std::lock_guard<std::mutex> lock(exposure_update_mutex_);
      requested_ms = pending_exposure_time_ms_;
    }

    cameraRuntimeParameters runtime{};
    if (cameraGetRuntimeParameters(camera_, &runtime) != HSI_OK) {
      RCLCPP_ERROR(get_logger(), "Failed to read runtime parameters while applying exposure update");
      return;
    }

    const double previous_exposure_ms = exposure_time_ms_;
    runtime.exposure_time_ms = requested_ms;
    runtime.gain = gain_;
    runtime.frame_rate_hz = acquisition_rate_hz_;
    runtime.trigger_mode = free_run_ ? TM_NoTriggering : TM_SoftwareTriggered;

    auto try_reinit_rollback = [&](const char * reason) {
      RCLCPP_ERROR(
        get_logger(),
        "%s for exposure update to %.6f ms; rolling back to %.6f ms",
        reason,
        requested_ms,
        previous_exposure_ms);
      cameraRuntimeParameters rb{};
      if (cameraGetRuntimeParameters(camera_, &rb) != HSI_OK) {
        RCLCPP_ERROR(get_logger(), "Rollback: could not read runtime parameters");
        return;
      }
      rb.exposure_time_ms = previous_exposure_ms;
      rb.gain = gain_;
      rb.frame_rate_hz = acquisition_rate_hz_;
      rb.trigger_mode = free_run_ ? TM_NoTriggering : TM_SoftwareTriggered;
      (void)cameraStop(camera_);
      camera_started_ = false;
      if (full_reinit_from_idle(rb, "rollback previous exposure")) {
        publish_exposure_time_actual();
        RCLCPP_INFO(
          get_logger(),
          "Rolled back to exposure_time_ms=%.6f (camera reinitialized)",
          previous_exposure_ms);
      } else {
        RCLCPP_ERROR(get_logger(), "Rollback reinit failed; fix requires node restart or camera re-plug");
      }
    };

    bool used_stop_to_idle = false;
    if (camera_started_) {
      const HSI_RETURN pause_err = cameraPause(camera_);
      if (pause_err == HSI_OK) {
        camera_started_ = false;
        RCLCPP_INFO(get_logger(), "Exposure update: cameraPause OK (STARTED->READY)");
      } else {
        RCLCPP_WARN(
          get_logger(),
          "cameraPause failed (%d, %s); using cameraStop+reinit (STARTED/READY->IDLE)",
          pause_err,
          hsi_return_name(pause_err));
        const HSI_RETURN stop_err = cameraStop(camera_);
        if (stop_err != HSI_OK) {
          RCLCPP_ERROR(
            get_logger(), "cameraStop failed during exposure update (%d, %s)", stop_err,
            hsi_return_name(stop_err));
          return;
        }
        camera_started_ = false;
        used_stop_to_idle = true;
      }
    }

    bool ok = false;
    if (used_stop_to_idle) {
      ok = full_reinit_from_idle(runtime, "exposure update (after Stop)");
    } else {
      const HSI_RETURN set_err = cameraSetRuntimeParameters(camera_, runtime);
      if (set_err != HSI_OK) {
        RCLCPP_ERROR(
          get_logger(), "cameraSetRuntimeParameters failed for exposure %.6f ms (%d, %s)", requested_ms,
          set_err, hsi_return_name(set_err));
        try_reinit_rollback("cameraSetRuntimeParameters failed");
        return;
      }

      if (!camera_start_with_retry("exposure update (after Pause)")) {
        RCLCPP_WARN(get_logger(), "cameraStart after Pause failed; trying Stop+reinit as recovery");
        const HSI_RETURN stop_err2 = cameraStop(camera_);
        if (stop_err2 != HSI_OK) {
          RCLCPP_ERROR(
            get_logger(), "recovery cameraStop failed (%d, %s)", stop_err2, hsi_return_name(stop_err2));
          try_reinit_rollback("recovery cameraStop failed");
          return;
        }
        camera_started_ = false;
        ok = full_reinit_from_idle(runtime, "exposure update (recovery after failed Start)");
      } else {
        ok = true;
      }
    }

    if (!ok) {
      try_reinit_rollback("Camera restart failed");
      return;
    }

    exposure_time_ms_ = requested_ms;
    publish_exposure_time_actual();
    RCLCPP_INFO(get_logger(), "Applied exposure_time_setpoint: %.6f ms (camera restarted)", exposure_time_ms_);
  }

  bool free_run_{true};
  double acquisition_rate_hz_{20.0};
  double throttled_rate_hz_{1.0};
  std::string device_name_;
  double exposure_time_ms_{10.0};
  double gain_{1.0};
  int test_frame_count_{-1};
  int camera_start_delay_ms_{0};
  int raw_mono_value_max_{1023};
  std::string mission_state_topic_;
  std::string run_name_path_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_throttled_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr raw_metadata_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr temp_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr run_dir_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr exposure_actual_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr exposure_setpoint_sub_;

  HANDLE camera_{nullptr};
  HANDLE camera_xiapi_{nullptr}; // requires separate pointer than camera
  FrameDataFormat frame_format_{};
  FrameFloat frame_{};
  bool camera_started_{false};

  std::thread worker_thread_;
  std::atomic<bool> stop_requested_{false};
  rclcpp::TimerBase::SharedPtr timer_;
  std::mutex mutex_;
  CommandType camera_command_request_{CommandType::kIdle};
  std::mutex command_state_mutex_;
  std::condition_variable command_state_cv_;
  bool command_requested_{false};
  bool command_completed_{false};
  bool command_ok_{false};
  bool timelapse_state_{false};
  bool timelapse_tick_{false};
  bool auto_exposure_running_{false};
  std::string command_status_message_;
  std::mutex exposure_update_mutex_;
  std::atomic<bool> exposure_update_pending_{false};
  double pending_exposure_time_ms_{0.0};
  rclcpp::Service<hsi_camera_driver_cpp::srv::CameraCommands>::SharedPtr camera_commands_srv_;

  std::filesystem::path camera_dir_;
  std::filesystem::path raw_dir_;
  std::filesystem::path context_dir_;
  std::filesystem::path dark_dir_;
  std::filesystem::path white_dir_;
  std::filesystem::path non_uniformity_dir_;
  std::filesystem::path calibration_dir_;
  std::filesystem::path test_image_dir_;
  std::filesystem::path optical_setup_dir_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<HsiCameraAcquisitionNode>();
    rclcpp::spin(node);
  } catch (const std::exception & ex) {
    std::fprintf(stderr, "Fatal error: %s\n", ex.what());
  }
  rclcpp::shutdown();
  return 0;
}
