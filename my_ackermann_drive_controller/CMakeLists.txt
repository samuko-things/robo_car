cmake_minimum_required(VERSION 3.8)
project(my_ackermann_drive_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)



############# CPP FILES ##################################
# Include Cpp "include" directory
include_directories(include)

# Create Cpp executable
add_executable(my_ackermann_drive_controller src/my_ackermann_drive_controller.cpp)
ament_target_dependencies(my_ackermann_drive_controller rclcpp std_msgs geometry_msgs ackermann_msgs sensor_msgs nav_msgs tf2 tf2_ros)


# Install Cpp executables
install(TARGETS
    my_ackermann_drive_controller
    DESTINATION lib/${PROJECT_NAME}
)
###################################################################



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
