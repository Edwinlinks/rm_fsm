cmake_minimum_required(VERSION 3.10)
project(rm_localization)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        rm_common
        roscpp
        roslint
        std_msgs
        rm_msgs
        tf
        apriltag_ros
        )


include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

catkin_package(
        INCLUDE_DIRS
        include
        CATKIN_DEPENDS
        roscpp
        roslint
        tf
        rm_msgs
        rm_common
        apriltag_ros
)

#rm_localization
add_executable(rm_localization src/localization/rm_local_node.cpp src/localization/rm_localization.cpp)
target_link_libraries(rm_localization ${catkin_LIBRARIES})
