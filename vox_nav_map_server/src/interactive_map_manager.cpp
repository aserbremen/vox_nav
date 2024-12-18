#include <vox_nav_map_server/interactive_map_manager.hpp>

namespace vox_nav_map_server {

InteractiveMapManager::InteractiveMapManager(const rclcpp::NodeOptions& options)
    : Node("interactive_map_manager", options) {
  RCLCPP_INFO(this->get_logger(), "Creating..");
  // initialize shared pointers asap
  original_octomap_msg_ = std::make_shared<octomap_msgs::msg::Octomap>();
  collision_octomap_msg_ = std::make_shared<octomap_msgs::msg::Octomap>();
  elevated_surfel_octomap_msg_ = std::make_shared<octomap_msgs::msg::Octomap>();
  elevated_surfel_poses_msg_ = std::make_shared<geometry_msgs::msg::PoseArray>();
  octomap_pointcloud_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
  traversable_pointcloud_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
  non_traversable_pointcloud_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
  elevated_surfels_pointcloud_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
  original_octomap_markers_msg_ = std::make_shared<visualization_msgs::msg::MarkerArray>();
  elevated_surfel_octomap_markers_msg_ = std::make_shared<visualization_msgs::msg::MarkerArray>();
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  static_transform_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  // Declare and set parameters
  pointcloud_topic_ = declare_parameter("pointcloud_topic", "mrg_slam/map_points_service");
}

}  // namespace vox_nav_map_server

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(vox_nav_map_server::InteractiveMapManager)