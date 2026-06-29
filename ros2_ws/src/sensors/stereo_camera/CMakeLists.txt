cmake_minimum_required(VERSION 3.8)
project(stereo_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(stereo_camera src/stereo_camera.cpp)
target_compile_features(stereo_camera PUBLIC cxx_std_17)

ament_target_dependencies(stereo_camera
  rclcpp
  sensor_msgs
)

target_link_libraries(stereo_camera ${OpenCV_LIBS})
target_include_directories(stereo_camera PRIVATE ${OpenCV_INCLUDE_DIRS})

install(TARGETS stereo_camera
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY camera_calibration/cal_results/
  DESTINATION share/${PROJECT_NAME}/cal_results)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
