cmake_minimum_required(VERSION 3.21)

get_filename_component(CURRENT_DIR_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
string(TOLOWER ${CURRENT_DIR_NAME} PROJECT)
project(${PROJECT})

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compile commands for IDE support
# set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")

include(requirements.cmake)

file(GLOB_RECURSE SOURCES "src/*.cpp") 
add_executable(rosmaster_translator ${SOURCES})

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  adore_dynamics
  std_msgs
  adore_map
  adore_dynamics_conversions
)

target_include_directories(${PROJECT} PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

file(GLOB_RECURSE TEST_SOURCES test/*.cpp)
if(TEST_SOURCES)
    ament_add_gtest(${PROJECT}_test ${TEST_SOURCES})
    if(TARGET ${PROJECT}_test)
        ament_target_dependencies(${PROJECT}_test 
            rclcpp 
            std_msgs 
            adore_ros2_msgs
        )
    endif()
endif()

install(TARGETS
    ${PROJECT}
  DESTINATION lib/${PROJECT}
)

ament_package()
