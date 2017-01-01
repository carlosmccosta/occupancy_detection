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
		const std::string& _configuration_namespace, pcl::CropBox<pcl::PointXYZRGB>::Ptr _filter) {

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

} /* namespace filters */
} /* namespace occupancy_detection */
