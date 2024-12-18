#pragma once

#include <octomap/octomap.h>
#include <octomap/octomap_utils.h>
#include <octomap_msgs/conversions.h>
#include <pcl/common/common.h>
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/pose_array.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <pcl_ros/transforms.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>
#include <vox_nav_msgs/msg/oriented_nav_sat_fix.hpp>
#include <vox_nav_msgs/srv/get_traversability_map.hpp>
#include <vox_nav_utilities/map_manager_helpers.hpp>
#include <vox_nav_utilities/pcl_helpers.hpp>
#include <vox_nav_utilities/tf_helpers.hpp>
// #include <mutex>

namespace vox_nav_map_server {

/**
 * @brief This node subcribes to a point cloud topic and publishes all necessary
 * messages navigate a rover.
 *
 */
class InteractiveMapManager : public rclcpp::Node {
 public:
  using PointType = pcl::PointXYZRGB;
  // the following structs can be used somewhere general in order to avoid duplicate code
  struct PCDPreProcessingParams {
    double pcd_map_downsample_voxel_size;
    int remove_outlier_mean_K;
    double remove_outlier_stddev_threshold;
    double remove_outlier_radius_search;
    int remove_outlier_min_neighbors_in_radius;
    bool apply_filters;
    PCDPreProcessingParams()
        : pcd_map_downsample_voxel_size(0.1),
          remove_outlier_mean_K(10),
          remove_outlier_stddev_threshold(0.1),
          remove_outlier_radius_search(0.1),
          remove_outlier_min_neighbors_in_radius(1),
          apply_filters(false) {}
  };

  struct CostRegressionParams {
    double uniform_sample_radius;
    double surfel_radius;
    double max_allowed_tilt;
    double max_allowed_point_deviation;
    double max_allowed_energy_gap;
    double node_elevation_distance;
    double plane_fit_threshold;
    double robot_mass;
    double average_speed;
    double max_color_range;
    std::vector<double> cost_critic_weights;
    CostRegressionParams()
        : uniform_sample_radius(0.2),
          surfel_radius(0.1),
          max_allowed_tilt(10),
          max_allowed_point_deviation(0.1),
          max_allowed_energy_gap(0.1),
          node_elevation_distance(1),
          plane_fit_threshold(10),
          robot_mass(0.1),
          average_speed(0.1),
          max_color_range(255.0),
          cost_critic_weights({0.33, 0.33, 0.33}) {}
  };

  /**
   * @brief Construct a new Interactive Map Manager object
   *
   * @param options rclcpp::NodeOptions needed to register a component with ROS2
   */
  InteractiveMapManager(const rclcpp::NodeOptions& options = rclcpp::NodeOptions().use_intra_process_comms(true));

  ~InteractiveMapManager();

  /**
   * @brief Callback function for point cloud subscription
   *
   * @param msg Point cloud message
   */
  void pointcloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  /**
   * @brief Given preprocessed and cost regressed point cloud of PCD Map
   * this methed, constructs an octomap from this point cloud.
   * Markers representing Octomap are also filled within this function
   *
   */
  void handleOriginalOctomap();

  /**
   * @brief Given preprocessed(denoise, rigid body trans. etc.) point cloud,
   * regresses costs to original point cloud based on features extracted from surfels
   * These are also know as cost critics.
   * e.g, tilt, max energy differnece, average point deviation from surfel plane etc.
   *
   */
  void regressCosts();

  /**
   * @brief once map is georefenced, this function
   *  is called from timerCallback to publish map related visuals
   *  e.g point cloud, octomap markers etc
   *
   */
  void publishMapVisuals();

  /**
   * @brief It is possible to apply some preprocessing steps to original PCD map.
   * Noise removal , downsampling , rigid body transfroms etc.
   * Look at the params.yaml for filter related paramaters
   *
   */
  void preProcessPCDMap();

  /**
   * @brief Get the Get Maps And Surfels Callback object, Service callback to
   * provide maps and surfels managed and cnfigured by this node
   *
   * @param request_header
   * @param request
   * @param response
   */
  void getGetTraversabilityMapCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                                       const std::shared_ptr<vox_nav_msgs::srv::GetTraversabilityMap::Request> request,
                                       std::shared_ptr<vox_nav_msgs::srv::GetTraversabilityMap::Response> response);

 private:
  // point cloud sub, we listen to this point cloud to create the traversability map
  std::string pointcloud_topic_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;

  // Service to provide Octomap, elevated surfel and elevated surfel poses
  rclcpp::Service<vox_nav_msgs::srv::GetTraversabilityMap>::SharedPtr get_traversability_map_service_;
  // publishes octomap in form of a point cloud message
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr octomap_pointloud_publisher_;
  // publishes octomap in form of a point cloud message
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr elevated_surfel_pcl_publisher_;
  // publishes octomap in form of a point cloud message
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr traversable_pointcloud_publisher_;
  // publishes octomap in form of a point cloud message
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr non_traversable_pointcloud_publisher_;
  // publish sampled node poses for planner to use.
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr octomap_markers_publisher_;
  // publish sampled node poses for planner to use.
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr elevated_surfel_octomap_markers_publisher_;

  // reusable octomap point loud message, dont need to recreate each time we publish
  sensor_msgs::msg::PointCloud2::SharedPtr octomap_pointcloud_msg_;
  // reusable octomap point loud message, dont need to recreate each time we publish
  sensor_msgs::msg::PointCloud2::SharedPtr elevated_surfels_pointcloud_msg_;
  // reusable octomap point loud message, dont need to recreate each time we publish
  sensor_msgs::msg::PointCloud2::SharedPtr traversable_pointcloud_msg_;
  // reusable octomap point loud message, dont need to recreate each time we publish
  sensor_msgs::msg::PointCloud2::SharedPtr non_traversable_pointcloud_msg_;
  // reusable octomap marker array message, used to publish occupied nodes onlyu
  visualization_msgs::msg::MarkerArray::SharedPtr original_octomap_markers_msg_;
  // reusable octomap marker array message, used to publish occupied nodes onlyu
  visualization_msgs::msg::MarkerArray::SharedPtr elevated_surfel_octomap_markers_msg_;
  // octomap acquired from original PCD map
  octomap_msgs::msg::Octomap::SharedPtr original_octomap_msg_;

  octomap_msgs::msg::Octomap::SharedPtr collision_octomap_msg_;
  // Surfels centers are elevated by node_elevation_distance_, and are stored in this
  // octomap, this maps is used by planner to sample states that are
  // strictly laying on ground but not touching. So it constrains the path to be on ground
  // while it can elevate thorogh ramps or slopes
  octomap_msgs::msg::Octomap::SharedPtr elevated_surfel_octomap_msg_;
  // it is also required to have orientation information of surfels, they are kept in
  // elevated_surfel_poses_msg_
  geometry_msgs::msg::PoseArray::SharedPtr elevated_surfel_poses_msg_;
  // otree object to read and store binary octomap from disk
  // rclcpp parameters from yaml file: full path to octomap file in disk
  // std::string pcd_map_filename_; // not needed
  // Pointcloud map is stroed here
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pure_traversable_pointcloud_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pure_non_traversable_pointcloud_;

  //  Pointcloud map is stroed here
  pcl::PointCloud<pcl::PointSurfel>::Ptr elevated_surfel_pointcloud_;
  // rclcpp parameters from yaml file: topic name for published octomap
  std::string octomap_publish_topic_name_;
  // rclcpp parameters from yaml file: topic name for published octomap as cloud
  std::string octomap_point_cloud_publish_topic_;
  std::string octomap_markers_publish_topic_;
  std::string traversable_pointcloud_publish_topic_;
  std::string non_traversable_pointcloud_publish_topic_;
  // rclcpp parameters from yaml file: frame id for map typicall: "map"
  std::string map_frame_id_;
  // rclcpp parameters from yaml file: voxel size for octomap
  double octomap_voxel_size_;
  // rclcpp parameters from yaml file: publish frequncy to publish map and transfroms
  int octomap_publish_frequency_;
  // rclcpp parameters from yaml file: if true, a cloud will be published which represents octomap
  bool publish_octomap_visuals_;
  // tf buffer to get access to transfroms
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_transform_broadcaster_;
  // Optional rigid body transform to apply to the cloud, if cloud
  // is depth camera frames we need to pull cloud back to conventional ROS frames
  vox_nav_utilities::RigidBodyTransformation pcd_map_transform_matrix_;
  //  see the struct, it is used to keep preprocess params orginzed
  PCDPreProcessingParams preprocess_params_;
  //  see the struct, it is used to keep cost regression params orginzed
  CostRegressionParams cost_params_;
  // hther map has beene configured yet
  volatile bool map_configured_;
  // we need to align static map to map only once, since it is static !
  std::once_flag configure_map_once_;
};

}  // namespace vox_nav_map_server
