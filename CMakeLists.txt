cmake_minimum_required(VERSION 2.8.3)
project(occupancy_detection)


#######################################################################################################################
## packages
#######################################################################################################################

set(${PROJECT_NAME}_CATKIN_COMPONENTS
    cmake_modules
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
    xmlrpcpp
)

find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_COMPONENTS})
find_package(Boost REQUIRED)
find_package(Eigen REQUIRED)
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
        Eigen
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
    ${Eigen_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


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


#==============================================================================
# library link
#==============================================================================

target_link_libraries(occupancy_detection
#    ${Boost_LIBRARIES}
#    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
)


#==============================================================================
# executables link
#==============================================================================

target_link_libraries(occupancy_detection_node
    occupancy_detection
#    ${Boost_LIBRARIES}
#    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
)

