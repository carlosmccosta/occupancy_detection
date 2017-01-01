/**\file occupancy_detection.cpp
 * \brief Description...
 *
 * @version 1.0
 * @author carloscosta
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <occupancy_detection/occupancy_detection.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace occupancy_detection {
// =============================================================================  <public-section>  ============================================================================
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
OccupancyDetection::OccupancyDetection() :
		tf2_buffer_(ros::Duration(10)), tf2_transform_listener_(tf2_buffer_) {}
OccupancyDetection::~OccupancyDetection() {}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <member-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
bool OccupancyDetection::loadConfigurationFromParameterServer(ros::NodeHandlePtr& _node_handle, ros::NodeHandlePtr& _private_node_handle, const std::string& _configuration_namespace) {
	node_handle_ = _node_handle;
	private_node_handle_ = _private_node_handle;

	private_node_handle_->param("map_frame_id", map_frame_id_, std::string("map"));
	private_node_handle_->param("sensor_pointcloud_subscribe_topics", topics_sensor_pointcloud_, std::string("/camera/depth_registered/points"));

	XmlRpc::XmlRpcValue detectors;
	if (_private_node_handle->getParam(_configuration_namespace, detectors) && detectors.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
		for (XmlRpc::XmlRpcValue::iterator it = detectors.begin(); it != detectors.end(); ++it) {
			std::string detector_name = it->first;
			PerceptionInterface::Ptr detector_ptr;
			if (detector_name.find("static_occupancy_detector") != std::string::npos) {
				detector_ptr.reset(new StaticOccupancyDetection());
			} else if (detector_name.find("dynamic_occupancy_detector") != std::string::npos) {
				detector_ptr.reset(new DynamicOccupancyDetection());
			}

			if (detector_ptr && detector_ptr->loadConfigurationFromParameterServer(_node_handle, _private_node_handle, _configuration_namespace + detector_name + "/")) {
				occupancy_detectors_.push_back(detector_ptr);
			}
		}
	}

	return !occupancy_detectors_.empty();
}

void OccupancyDetection::start() {
	pointcloud_subscribers_.clear();
	if (!topics_sensor_pointcloud_.empty()) {
		std::replace(topics_sensor_pointcloud_.begin(), topics_sensor_pointcloud_.end(), '+', ' ');

		std::stringstream ss(topics_sensor_pointcloud_);
		std::string topic_name;

		while (ss >> topic_name && !topic_name.empty()) {
			pointcloud_subscribers_.push_back(node_handle_->subscribe(topic_name, 1, &OccupancyDetection::processSensorPointCloud, this));
			ROS_INFO_STREAM("Adding " << topic_name << " to the list of sensor_msgs::PointCloud2 topics to use for occupancy detection");
		}
	}

	if (pointcloud_subscribers_.empty())
	{
		ROS_FATAL("Sensor point cloud topics must be provided!");
	} else {
		ros::spin();
	}
}

void OccupancyDetection::processSensorPointCloud(const sensor_msgs::PointCloud2ConstPtr& _cloud_msg) {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr sensor_pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	msg_conversions::convertSensorMsgToPCL(_cloud_msg, *sensor_pointcloud);
	if (transformCloudToMapFrame(*sensor_pointcloud, _cloud_msg->header.stamp)) {
		for (size_t i = 0; i < occupancy_detectors_.size(); ++i) {
			occupancy_detectors_[i]->processSensorData(sensor_pointcloud);
		}
	} else {
		ROS_WARN_STREAM("Discarding point cloud because there is no TF between [" << map_frame_id_ << " -> " << sensor_pointcloud->header.frame_id << "]");
	}
}

bool OccupancyDetection::transformCloudToMapFrame(pcl::PointCloud<pcl::PointXYZRGB>& pointcloud, const ros::Time& timestamp, const ros::Duration& timeout) {
	try {
		geometry_msgs::TransformStamped tf = tf2_buffer_.lookupTransform(map_frame_id_, pointcloud.header.frame_id, timestamp, timeout);
		pcl::transformPointCloud(pointcloud, pointcloud, msg_conversions::transformMsgToEigenTransform<float>(tf));
		pointcloud.header.frame_id = map_frame_id_;
		return true;
	} catch (...) {
		return false;
	}
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </member-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>  =======================================================================


} /* namespace occupancy_detection */
