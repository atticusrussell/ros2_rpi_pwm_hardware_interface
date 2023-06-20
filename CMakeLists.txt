cmake_minimum_required(VERSION 3.8)
project(rpi_pwm_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

################################################################################
### Find the pigpio shared libraries.
################################################################################
	
# Find the pigpio libraries.
find_library(PIGPIO_LIB 
	NAMES libpigpio.so
	HINTS /usr/local/lib
  REQUIRED)

find_library(PIGPIO_IF2_LIB 
	NAMES libpigpiod_if2.so
	HINTS /usr/local/lib
  REQUIRED)

################################################################################
################################################################################

# find_library(PIGPIO_IF2_LIB pigpiod_if2 REQUIRED)
# find_library(PIGPIO_LIB pigpio REQUIRED)

add_library(
  rpi_pwm_hardware_interface
  SHARED
  src/rpi_pwm_hardware_interface.cpp
  src/angular_servo.cpp
)
target_include_directories(
  rpi_pwm_hardware_interface
  PUBLIC
  include
)

# Add PIGPIO to the target_link_libraries
target_link_libraries(rpi_pwm_hardware_interface ${PIGPIO_IF2_LIB})
target_link_libraries(rpi_pwm_hardware_interface ${PIGPIO_LIB})

ament_target_dependencies(
  rpi_pwm_hardware_interface
  hardware_interface
  rclcpp
  rclcpp_lifecycle
)
# prevent pluginlib from using boost
target_compile_definitions(rpi_pwm_hardware_interface PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(
  hardware_interface rpi_pwm_hardware_interface.xml)

install(
  TARGETS
  rpi_pwm_hardware_interface
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # uncomment the line when a copyright and license is added to all source files
#   # set(ament_cmake_copyright_FOUND TRUE)
#   # the following line enables  cpplint (only works in a git repo)
#   # set(ament_cmake_cpplint_FOUND TRUE)
#   # ament_lint_auto_find_test_dependencies()

#   # find_package(ament_cmake_gmock REQUIRED)
#   # find_package(ros2_control_test_assets REQUIRED)

#   # ament_add_gmock(test_rpi_pwm_hardware_interface test/test_rpi_pwm_hardware_interface.cpp)
#   # target_include_directories(test_rpi_pwm_hardware_interface PRIVATE include)
#   # ament_target_dependencies(
#   #   test_rpi_pwm_hardware_interface
#   #   hardware_interface
#   #   pluginlib
#   #   ros2_control_test_assets
#   # )
# endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  rpi_pwm_hardware_interface
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()
