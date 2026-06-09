find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_lint_auto REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

find_package(adore_dynamics REQUIRED)
find_package(adore_map REQUIRED)
find_package(adore_dynamics_conversions REQUIRED)


# required for unit testing with gtest
find_package(ament_cmake_gtest REQUIRED)


# required to use ADORe messages 
find_package(adore_ros2_msgs REQUIRED)

