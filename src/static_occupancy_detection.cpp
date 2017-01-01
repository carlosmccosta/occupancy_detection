/**\file static_occupancy_detection.cpp
 * \brief Description...
 *
 * @version 1.0
 * @author carloscosta
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <occupancy_detection/static_occupancy_detection.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace occupancy_detection {

// =============================================================================  <public-section>  ============================================================================
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
StaticOccupancyDetection::StaticOccupancyDetection() :
		keep_detecting_when_objects_remain_in_roi_(true),
		detection_successful_in_last_sensor_data_(false),
		detection_failed_after_last_accepted_detection_(true)
		{}
StaticOccupancyDetection::~StaticOccupancyDetection() {}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <member-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
bool StaticOccupancyDetection::loadConfigurationFromParameterServer(ros::NodeHandlePtr& _node_handle, ros::NodeHandlePtr& _private_node_handle, const std::string& _configuration_namespace) {
	DynamicOccupancyDetection::loadConfigurationFromParameterServer(_node_handle, _private_node_handle, _configuration_namespace);
	double minimum_detection_time;
	_private_node_handle->param(_configuration_namespace + "minimum_detection_time", minimum_detection_time, 0.5);
	minimum_detection_time_.fromSec(minimum_detection_time);
	_private_node_handle->param(_configuration_namespace + "keep_detecting_when_objects_remain_in_roi", keep_detecting_when_objects_remain_in_roi_, true);
	return true;
}

bool StaticOccupancyDetection::detectOccupancy(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& _sensor_data) {
	if (DynamicOccupancyDetection::detectOccupancy(_sensor_data)) {
		if (detection_successful_in_last_sensor_data_) {
			ros::Duration elapsed_time_since_first_detection = pcl_conversions::fromPCL(_sensor_data->header.stamp) - last_detection_time_;
			if (elapsed_time_since_first_detection > minimum_detection_time_) {
				last_detection_time_ = pcl_conversions::fromPCL(_sensor_data->header.stamp);
				bool return_status = true;
				if (!keep_detecting_when_objects_remain_in_roi_ && !detection_failed_after_last_accepted_detection_) {
					return_status = false;
				}
				detection_failed_after_last_accepted_detection_ = false;
				return return_status;
			}
		} else {
			detection_successful_in_last_sensor_data_ = true;
			last_detection_time_ = pcl_conversions::fromPCL(_sensor_data->header.stamp);
		}
	} else {
		detection_successful_in_last_sensor_data_ = false;
		detection_failed_after_last_accepted_detection_ = true;
	}
	return false;
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </member-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>  =======================================================================

} /* namespace occupancy_detection */

