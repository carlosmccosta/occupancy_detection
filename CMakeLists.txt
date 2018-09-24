cmake_minimum_required(VERSION 2.8.3)
project(occupancy_detection)


#######################################################################################################################
## packages
#######################################################################################################################

set(${PROJECT_NAME}_CATKIN_COMPONENTS
    geometry_msgs
    pcl_conversions
    pcl_ros
    rosconsole
    roscpp
    rostime
    sensor_msgs
    std_msgs
    tf2
    tf2_ros
    visualization_msgs
    xmlrpcpp
)


find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_COMPONENTS})
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(PCL REQUIRED)



#######################################################################################################################
## catkin specific configuration
#######################################################################################################################

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        occupancy_detection
    CATKIN_DEPENDS
        ${${PROJECT_NAME}_CATKIN_COMPONENTS}
    DEPENDS
        Boost
        EIGEN3
        PCL
)



#######################################################################################################################
## build
#######################################################################################################################

#==============================================================================
# includes
#==============================================================================

include_directories(
    include
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)


#==============================================================================
# libraries
#==============================================================================

add_library(occupancy_detection
    src/filters.cpp
    src/static_occupancy_detection.cpp
    src/dynamic_occupancy_detection.cpp
    src/occupancy_detection.cpp
)


#==============================================================================
# executables
#==============================================================================

add_executable(occupancy_detection_node
    src/occupancy_detection_node.cpp
)



#==================
# dependencies
#==================

add_dependencies(occupancy_detection
    ${catkin_EXPORTED_TARGETS}
)

add_dependencies(occupancy_detection_node
    ${catkin_EXPORTED_TARGETS}
)


#==============================================================================
# library link
#==============================================================================

target_link_libraries(occupancy_detection
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
)


#==============================================================================
# executables link
#==============================================================================

target_link_libraries(occupancy_detection_node
    occupancy_detection
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
)
