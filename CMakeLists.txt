cmake_minimum_required(VERSION 2.8.3)
project(carmen2bag)

find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        nav_msgs
        roscpp
        rospy
        sensor_msgs
        std_msgs
        rosbag
        tf
        )

include_directories(include)

include_directories(
        ${catkin_INCLUDE_DIRS}
)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Werror")

add_executable(carmen2bag src/${SOURCE_FILES} src/converter.cpp src/main.cpp)
target_link_libraries(carmen2bag ${catkin_LIBRARIES})

