#pragma once

/**\file filters.h
 * \brief Description...
 *
 * @version 1.0
 * @author carloscosta
 */
 

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>  <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// std includes
#include <string>

// ROS includes
#include <ros/ros.h>
#include <ros/duration.h>
#include <ros/time.h>
#include <visualization_msgs/Marker.h>

// PCL includes
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>

// external libs includes
#include <boost/smart_ptr/shared_ptr.hpp>
#include <Eigen/Core>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace occupancy_detection {

// ################################################################################   filters   ################################################################################
namespace filters {

bool loadCropBoxFilterFromParameterServer(ros::NodeHandlePtr& _node_handle, ros::NodeHandlePtr& _private_node_handle,
		const std::string& _configuration_namespace, pcl::CropBox<pcl::PointXYZRGB>::Ptr& _filter);

bool loadBoxMarkerFromCropBox(pcl::CropBox<pcl::PointXYZRGB>::Ptr& _filter, visualization_msgs::MarkerPtr& _marker);
} /* namespace filters */
} /* namespace occupancy_detection */
