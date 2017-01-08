/**\file filters.cpp
 * \brief Description...
 *
 * @version 1.0
 * @author carloscosta
 */
 

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <occupancy_detection/filters.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace occupancy_detection {

namespace filters {

bool loadCropBoxFilterFromParameterServer(ros::NodeHandlePtr& _node_handle, ros::NodeHandlePtr& _private_node_handle,
		const std::string& _configuration_namespace, pcl::CropBox<pcl::PointXYZRGB>::Ptr& _filter) {

	double box_min_x, box_min_y, box_min_z;
	_private_node_handle->param(_configuration_namespace + "box_min_x", box_min_x, -10.0);
	_private_node_handle->param(_configuration_namespace + "box_min_y", box_min_y, -10.0);
	_private_node_handle->param(_configuration_namespace + "box_min_z", box_min_z, -10.0);
	_filter->setMin(Eigen::Vector4f(box_min_x, box_min_y, box_min_z, 1));

	double box_max_x, box_max_y, box_max_z;
	_private_node_handle->param(_configuration_namespace + "box_max_x", box_max_x, 10.0);
	_private_node_handle->param(_configuration_namespace + "box_max_y", box_max_y, 10.0);
	_private_node_handle->param(_configuration_namespace + "box_max_z", box_max_z, 10.0);
	_filter->setMax(Eigen::Vector4f(box_max_x, box_max_y, box_max_z, 1));

	double box_translation_x, box_translation_y, box_translation_z;
	_private_node_handle->param(_configuration_namespace + "box_translation_x", box_translation_x, 0.0);
	_private_node_handle->param(_configuration_namespace + "box_translation_y", box_translation_y, 0.0);
	_private_node_handle->param(_configuration_namespace + "box_translation_z", box_translation_z, 0.0);
	_filter->setTranslation(Eigen::Vector3f(box_translation_x, box_translation_y, box_translation_z));

	double box_rotation_roll, box_rotation_pitch, box_rotation_yaw;
	_private_node_handle->param(_configuration_namespace + "box_rotation_roll", box_rotation_roll, 0.0);
	_private_node_handle->param(_configuration_namespace + "box_rotation_pitch", box_rotation_pitch, 0.0);
	_private_node_handle->param(_configuration_namespace + "box_rotation_yaw", box_rotation_yaw, 0.0);
	_filter->setRotation(Eigen::Vector3f(box_rotation_roll, box_rotation_pitch, box_rotation_yaw));

	bool invert_selection;
	_private_node_handle->param(_configuration_namespace + "invert_selection", invert_selection, false);
	_filter->setNegative(invert_selection);

	return true;
}

bool loadBoxMarkerFromCropBox(pcl::CropBox<pcl::PointXYZRGB>::Ptr& _filter, visualization_msgs::MarkerPtr& _marker) {
	_marker->type = visualization_msgs::Marker::CUBE;
	_marker->action = visualization_msgs::Marker::ADD;
	Eigen::Vector4f min = _filter->getMin();
	Eigen::Vector4f max = _filter->getMax();
	Eigen::Vector3f translation = _filter->getTranslation();
	double dx = max(0) - min(0);
	double dy = max(1) - min(1);
	double dz = max(2) - min(2);
	_marker->pose.position.x = min(0) + dx / 2.0 + translation(0);
	_marker->pose.position.y = min(1) + dy / 2.0 + translation(1);
	_marker->pose.position.z = min(2) + dz / 2.0 + translation(2);
	_marker->pose.orientation.x = 0.0;
	_marker->pose.orientation.y = 0.0;
	_marker->pose.orientation.z = 0.0;
	_marker->pose.orientation.w = 1.0;
	_marker->scale.x = dx;
	_marker->scale.y = dy;
	_marker->scale.z = dz;
	_marker->lifetime = ros::Duration();
}

} /* namespace filters */
} /* namespace occupancy_detection */
