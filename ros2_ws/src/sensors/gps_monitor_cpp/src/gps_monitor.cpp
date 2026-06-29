#include <cstdio>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <string>

#include "com.h"
#include "gps.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"

namespace
{

double nmea_to_decimal_degrees(const char * nmea, char hemisphere)
{
  const double raw = std::atof(nmea);
  const int degrees = static_cast<int>(raw / 100.0);
  const double minutes = raw - (degrees * 100.0);
  double decimal = degrees + (minutes / 60.0);
  if (hemisphere == 'S' || hemisphere == 'W') {
    decimal = -decimal;
  }
  return decimal;
}

sensor_msgs::msg::NavSatFix to_nav_sat_fix(
  const rclcpp::Time & stamp,
  const std::string & frame_id)
{
  sensor_msgs::msg::NavSatFix msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
  msg.status.status = (Save_Data.Usefull_Flag == 1) ?
    sensor_msgs::msg::NavSatStatus::STATUS_FIX :
    sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
  if (Save_Data.Slatitude[0] != '\0' && Save_Data.Slongitude[0] != '\0') {
    msg.latitude = nmea_to_decimal_degrees(Save_Data.Slatitude, Save_Data.N_S[0]);
    msg.longitude = nmea_to_decimal_degrees(Save_Data.Slongitude, Save_Data.E_W[0]);
  } else {
    msg.latitude = std::numeric_limits<double>::quiet_NaN();
    msg.longitude = std::numeric_limits<double>::quiet_NaN();
  }
  msg.altitude = std::numeric_limits<double>::quiet_NaN();
  msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
  return msg;
}

void safe_print_gps_data()
{
  // print_Save() calls insert_array(), which crashes on empty lat/lon (no fix).
  std::printf("*****************************************************\n");
  std::printf("UTCTime\t\t:[%s]\n", Save_Data.UTCTime);
  std::printf("Slatitude\t:[%s]\n", Save_Data.Slatitude);
  std::printf("N/S\t\t:[%s]\n", Save_Data.N_S);
  std::printf("Slongitude\t:[%s]\n", Save_Data.Slongitude);
  std::printf("E/W\t\t:[%s]\n", Save_Data.E_W);
  std::printf("*****************************************************\n");
}

}  // namespace

class GpsMonitor : public rclcpp::Node
{
public:
  GpsMonitor()
  : Node("gps_monitor")
  {
    serial_port_ = declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
    frame_id_ = declare_parameter<std::string>("frame_id", "gps");
    topic_ = declare_parameter<std::string>("topic", "/sensors/gps");
    publish_invalid_ = declare_parameter<bool>("publish_invalid", false);

    publisher_ = create_publisher<sensor_msgs::msg::NavSatFix>(topic_, 10);

    if (publish_invalid_) {
      RCLCPP_INFO(
        get_logger(),
        "publish_invalid enabled: printing and publishing GPS data without a valid fix");
    }

    fd_ = open_port(const_cast<char *>(serial_port_.c_str()));
    if (fd_ < 0) {
      RCLCPP_FATAL(get_logger(), "Failed to open serial port %s", serial_port_.c_str());
      throw std::runtime_error("Failed to open serial port");
    } else {
      RCLCPP_INFO(get_logger(), "Opened Serial Port %s", serial_port_.c_str());
    }

    if (set_com_config(fd_, 115200, 8, 'N', 1) < 0) {
      RCLCPP_FATAL(get_logger(), "Failed to configure serial port");
      close(fd_);
      throw std::runtime_error("Failed to configure serial port");
    } else {
      RCLCPP_INFO(get_logger(), "Configured serial port");

    }

    timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&GpsMonitor::timer_callback, this));
  }

  ~GpsMonitor() override
  {
    if (fd_ >= 0) {
      close(fd_);
    }
  }

private:
  void timer_callback()
  {
    std::memset(read_buffer_, 0, BUFFER_SIZE);
    const int read_size = read_Buffer(fd_, read_buffer_);
    if (read_size <= 0) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Timer tick: no serial data from %s", serial_port_.c_str());
      return;
    }

    read_GPS_Data(read_buffer_);
    parse_GpsDATA();

    if (Save_Data.ParseData_Flag != 1) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Timer tick: no GPRMC/GNRMC sentence in buffer (%d bytes read)", read_size);
      return;
    }

    if (Save_Data.Usefull_Flag != 1 && !publish_invalid_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Timer tick: GPS sentence parsed but no valid fix");
      Save_Data.ParseData_Flag = 0;
      Save_Data.Usefull_Flag = 0;
      return;
    }

    if (publish_invalid_ && Save_Data.Usefull_Flag != 1) {
      safe_print_gps_data();
    }

    const auto message = to_nav_sat_fix(now(), frame_id_);
    publisher_->publish(message);

    Save_Data.Usefull_Flag = 0;
    Save_Data.ParseData_Flag = 0;

    if (message.status.status == sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
      RCLCPP_INFO(
        get_logger(), "Published NavSatFix lat=%.6f lon=%.6f",
        message.latitude, message.longitude);
    } else {
      RCLCPP_INFO(
        get_logger(), "Published NavSatFix (no fix) lat=%.6f lon=%.6f",
        message.latitude, message.longitude);
    }
  }

  std::string serial_port_;
  std::string frame_id_;
  std::string topic_;
  bool publish_invalid_{false};
  int fd_{-1};
  char read_buffer_[BUFFER_SIZE];
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<GpsMonitor>());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("gps_monitor"), "Exception: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
