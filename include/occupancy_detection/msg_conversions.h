#pragma once

/**\file msg_convertions.h
 * \brief Description...
 *
 * @version 1.0
 * @author carloscosta
 */
 

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>  <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// std includes
#include <string>
#include <vector>

// ROS includes
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>

// PCL includes
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl_conversions/pcl_conversions.h>

// external libs includes
#include <Eigen/Core>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


namespace occupancy_detection {

// #############################################################################   msg_convertions   ###########################################################################
namespace msg_conversions {

void convertSensorMsgToPCL(const sensor_msgs::PointCloud2ConstPtr& _cloud_msg_in, pcl::PointCloud<pcl::PointXYZRGB>& _sensor_data_out) {
	pcl::fromROSMsg(*_cloud_msg_in, _sensor_data_out);
	std::vector<int> indexes;
	pcl::removeNaNFromPointCloud(_sensor_data_out, _sensor_data_out, indexes);
}

template <typename Scalar>
Eigen::Transform<Scalar, 3, Eigen::Affine> transformMsgToEigenTransform(const geometry_msgs::TransformStamped& tf) {
	Eigen::Translation<Scalar, 3> translation(tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z);
	Eigen::Quaternion<Scalar> rotation(tf.transform.rotation.w, tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z);
	return Eigen::Transform<Scalar, 3, Eigen::Affine>(translation * rotation.toRotationMatrix());
}


} /* namespace msg_conversions */
} /* namespace occupancy_detection */
