cmake_minimum_required(VERSION 2.8.3)
project(pointcloud_accumulator)

find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS 
    geometry_msgs
    roscpp
    sensor_msgs
    tf2
    tf2_eigen
    tf2_ros
    tf2_geometry_msgs
    tf2_sensor_msgs
)
find_package(Boost REQUIRED COMPONENTS
    program_options
    system
    thread
)
set(ROS_COMPILE_FLAGS "-W -Wall -Wextra -pedantic")

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)
link_directories(
    ${catkin_LIBRARY_DIRS}
    ${PCL_LIBRARY_DIRS}
    ${Boost_LIBRARY_DIRS}
)
add_definitions(${PCL_DEFINITIONS})

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs tf2 tf2_eigen tf2_ros tf2_geometry_msgs tf2_sensor_msgs
    DEPENDS PCL Boost
)

add_executable(pointcloud_accumulator
    src/pointcloud_accumulator_node.cpp
    src/pointcloud_accumulator.cpp
)
target_link_libraries(pointcloud_accumulator
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)
