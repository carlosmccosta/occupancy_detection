/**\file dynamic_occupancy_detection.cpp
 * \brief Description...
 *
 * @version 1.0
 * @author carloscosta
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <occupancy_detection/dynamic_occupancy_detection.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace occupancy_detection {

// =============================================================================  <public-section>  ============================================================================
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
DynamicOccupancyDetection::DynamicOccupancyDetection() : minimum_number_of_sensor_points_inside_roi_(5) {}
DynamicOccupancyDetection::~DynamicOccupancyDetection() {}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <member-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
bool DynamicOccupancyDetection::loadConfigurationFromParameterServer(ros::NodeHandlePtr& _node_handle, ros::NodeHandlePtr& _private_node_handle, const std::string& _configuration_namespace) {
	std::string occupancy_topic;
	_private_node_handle->param(_configuration_namespace + "occupancy_publish_topic", occupancy_topic, std::string("occupancy_detection"));
	if (!occupancy_topic.empty()) {
		occupancy_publisher_ = _node_handle->advertise<geometry_msgs::PointStamped>(occupancy_topic, 1, true);
	}

	std::string filtered_pointcloud_publish_topic;
	_private_node_handle->param(_configuration_namespace + "filtered_pointcloud_publish_topic", filtered_pointcloud_publish_topic, std::string("filtered_pointcloud"));
	if (!filtered_pointcloud_publish_topic.empty()) {
		filtered_cloud_publisher_ = _node_handle->advertise<sensor_msgs::PointCloud2>(filtered_pointcloud_publish_topic, 1, true);
	}

	_private_node_handle->param(_configuration_namespace + "minimum_number_of_sensor_points_inside_roi", minimum_number_of_sensor_points_inside_roi_, 5);

	filters_.clear();
	typename pcl::Filter<pcl::PointXYZRGB>::Ptr filter_base(new pcl::CropBox<pcl::PointXYZRGB>());
	typename pcl::CropBox<pcl::PointXYZRGB>::Ptr filter = boost::static_pointer_cast< typename pcl::CropBox<pcl::PointXYZRGB> >(filter_base);
	filters::loadCropBoxFilterFromParameterServer(_node_handle, _private_node_handle, _configuration_namespace + "crop_box/", filter);
	filters_.push_back(filter_base);
	return true;
}

bool DynamicOccupancyDetection::processSensorData(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& _sensor_data) {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr sensor_data_filtered(new pcl::PointCloud<pcl::PointXYZRGB>());
	if (filterSensorData(_sensor_data, sensor_data_filtered)) {
		if (detectOccupancy(sensor_data_filtered)) {
			if (!occupancy_publisher_.getTopic().empty()) {
				occupancy_publisher_.publish(occupancy_detected_);
			}

			if (!filtered_cloud_publisher_.getTopic().empty()) {
				sensor_msgs::PointCloud2Ptr cloud_msg(new sensor_msgs::PointCloud2());
				pcl::toROSMsg(*sensor_data_filtered, *cloud_msg);
				filtered_cloud_publisher_.publish(cloud_msg);
			}
			return true;
		}
	}

	return false;
}

bool DynamicOccupancyDetection::filterSensorData(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& _sensor_data_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& _sensor_data_filtered) {
	for (size_t i = 0; i < filters_.size(); ++i) {
		if (i == 0) {
			filters_[i]->setInputCloud(_sensor_data_in);
			filters_[i]->filter(*_sensor_data_filtered);
		} else {
			filters_[i]->setInputCloud(_sensor_data_filtered);
			filters_[i]->filter(*_sensor_data_filtered);
		}
	}
	return !filters_.empty();
}

bool DynamicOccupancyDetection::detectOccupancy(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& _sensor_data) {
	if ((int)_sensor_data->size() > minimum_number_of_sensor_points_inside_roi_) {
		Eigen::Vector4d pointcloud_centroid;
		if ((int)pcl::compute3DCentroid(*_sensor_data, pointcloud_centroid) > minimum_number_of_sensor_points_inside_roi_) {
			occupancy_detected_.header.frame_id = _sensor_data->header.frame_id;
			occupancy_detected_.header.stamp = pcl_conversions::fromPCL(_sensor_data->header.stamp);
			occupancy_detected_.point.x = pointcloud_centroid(0);
			occupancy_detected_.point.y = pointcloud_centroid(1);
			occupancy_detected_.point.z = pointcloud_centroid(2);
			return true;
		}

	}

	return false;
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </member-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>  =======================================================================

} /* namespace occupancy_detection */
