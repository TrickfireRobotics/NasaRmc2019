cmake_minimum_required(VERSION 2.8.3)
project(tfr_aruco)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    actionlib
    roscpp
    tfr_msgs
    tf2
    cv_bridge
    image_geometry
    image_transport
    sensor_msgs
    message_generation
)

generate_messages(
    DEPENDENCIES sensor_msgs
)

find_package(OpenCV 3.4.6 REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tfr_communication
    CATKIN_DEPENDS roscpp message_runtime sensor_msgs
#  DEPENDS system_lib
)

include_directories(
  include/${PROJECT_NAME}
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(aruco_action_server src/aruco_action_server.cpp)
target_link_libraries(aruco_action_server ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(aruco_action_server ${catkin_EXPORTED_TARGETS})
