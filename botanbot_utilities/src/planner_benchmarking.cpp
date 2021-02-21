#include "botanbot_utilities/planner_benchmarking.hpp"

namespace botanbot_utilities
{
PlannerBenchMarking::PlannerBenchMarking()
: Node("planner_benchmarking_rclcpp_node")
{
  is_octomap_ready_ = false;
  octomap_msg_ = std::make_shared<octomap_msgs::msg::Octomap>();

  this->declare_parameter("selected_planners", std::vector<std::string>({"PRMstar"}));
  this->declare_parameter("planner_timeout", 5.0);
  this->declare_parameter("interpolation_parameter", 50);
  this->declare_parameter("octomap_topic", "octomap");
  this->declare_parameter("octomap_voxel_size", 0.2);
  this->declare_parameter("selected_state_space", "REEDS");
  this->declare_parameter("min_turning_radius", 2.5);
  this->declare_parameter("state_space_boundries.minx", -50.0);
  this->declare_parameter("state_space_boundries.maxx", 50.0);
  this->declare_parameter("state_space_boundries.miny", -10.0);
  this->declare_parameter("state_space_boundries.maxy", 10.0);
  this->declare_parameter("state_space_boundries.minz", -10.0);
  this->declare_parameter("state_space_boundries.maxz", 10.0);
  this->declare_parameter("state_space_boundries.minyaw", -3.14);
  this->declare_parameter("state_space_boundries.maxyaw", 3.14);
  this->declare_parameter("robot_body_dimens.x", 1.5);
  this->declare_parameter("robot_body_dimens.y", 1.5);
  this->declare_parameter("robot_body_dimens.z", 0.4);
  this->declare_parameter("start.x", 0.0);
  this->declare_parameter("start.y", 0.0);
  this->declare_parameter("start.z", 0.0);
  this->declare_parameter("start.yaw", 0.0);
  this->declare_parameter("goal.x", 0.0);
  this->declare_parameter("goal.y", 0.0);
  this->declare_parameter("goal.z", 0.0);
  this->declare_parameter("goal.yaw", 0.0);
  this->declare_parameter("goal_tolerance", 0.2);
  this->declare_parameter("num_benchmark_runs", 100);
  this->declare_parameter("max_memory", 2048);
  this->declare_parameter("results_output_file", "/home/user/get.log");
  this->declare_parameter("publish_a_sample_bencmark", true);
  this->declare_parameter("sample_bencmark_plans_topic", "benchmark_plan");


  this->get_parameter("selected_planners", selected_planners_);
  this->get_parameter("planner_timeout", planner_timeout_);
  this->get_parameter("interpolation_parameter", interpolation_parameter_);
  this->get_parameter("octomap_topic", octomap_topic_);
  this->get_parameter("octomap_voxel_size", octomap_voxel_size_);
  this->get_parameter("selected_state_space", selected_state_space_);
  this->get_parameter("min_turning_radius", min_turning_radius_);
  this->get_parameter("state_space_boundries.minx", se_bounds_.minx);
  this->get_parameter("state_space_boundries.maxx", se_bounds_.maxx);
  this->get_parameter("state_space_boundries.miny", se_bounds_.miny);
  this->get_parameter("state_space_boundries.maxy", se_bounds_.maxy);
  this->get_parameter("state_space_boundries.minz", se_bounds_.minz);
  this->get_parameter("state_space_boundries.maxz", se_bounds_.maxz);
  this->get_parameter("state_space_boundries.minyaw", se_bounds_.minyaw);
  this->get_parameter("state_space_boundries.maxyaw", se_bounds_.maxyaw);
  this->get_parameter("robot_body_dimens.x", robot_body_dimensions_.x);
  this->get_parameter("robot_body_dimens.y", robot_body_dimensions_.y);
  this->get_parameter("robot_body_dimens.z", robot_body_dimensions_.z);
  this->get_parameter("start.x", start_.x);
  this->get_parameter("start.y", start_.y);
  this->get_parameter("start.z", start_.z);
  this->get_parameter("start.yaw", start_.yaw);
  this->get_parameter("goal.x", goal_.x);
  this->get_parameter("goal.y", goal_.y);
  this->get_parameter("goal.z", goal_.z);
  this->get_parameter("goal.yaw", goal_.yaw);
  this->get_parameter("goal_tolerance", goal_tolerance_);
  this->get_parameter("num_benchmark_runs", num_benchmark_runs_);
  this->get_parameter("max_memory", max_memory_);
  this->get_parameter("results_output_file", results_output_file_);
  this->get_parameter("publish_a_sample_bencmark", publish_a_sample_bencmark_);
  this->get_parameter("sample_bencmark_plans_topic", sample_bencmark_plans_topic_);

  if (selected_state_space_ == "REEDS") {
    ompl_se_bounds_ = std::make_shared<ompl::base::RealVectorBounds>(2);
    ompl_se_bounds_->setLow(0, se_bounds_.minx);
    ompl_se_bounds_->setHigh(0, se_bounds_.maxx);
    ompl_se_bounds_->setLow(1, se_bounds_.miny);
    ompl_se_bounds_->setHigh(1, se_bounds_.maxy);
    ompl_se_bounds_->setLow(2, se_bounds_.minyaw);
    ompl_se_bounds_->setHigh(2, se_bounds_.maxyaw);
    state_space_ = std::make_shared<ompl::base::ReedsSheppStateSpace>(min_turning_radius_);
    state_space_->as<ompl::base::ReedsSheppStateSpace>()->setBounds(*ompl_se_bounds_);
  } else if (selected_state_space_ == "DUBINS") {
    ompl_se_bounds_ = std::make_shared<ompl::base::RealVectorBounds>(2);
    ompl_se_bounds_->setLow(0, se_bounds_.minx);
    ompl_se_bounds_->setHigh(0, se_bounds_.maxx);
    ompl_se_bounds_->setLow(1, se_bounds_.miny);
    ompl_se_bounds_->setHigh(1, se_bounds_.maxy);
    ompl_se_bounds_->setLow(2, se_bounds_.minyaw);
    ompl_se_bounds_->setHigh(2, se_bounds_.maxyaw);
    state_space_ = std::make_shared<ompl::base::DubinsStateSpace>(min_turning_radius_, true);
    state_space_->as<ompl::base::DubinsStateSpace>()->setBounds(*ompl_se_bounds_);
  } else if (selected_state_space_ == "SE2") {
    ompl_se_bounds_ = std::make_shared<ompl::base::RealVectorBounds>(2);
    ompl_se_bounds_->setLow(0, se_bounds_.minx);
    ompl_se_bounds_->setHigh(0, se_bounds_.maxx);
    ompl_se_bounds_->setLow(1, se_bounds_.miny);
    ompl_se_bounds_->setHigh(1, se_bounds_.maxy);
    ompl_se_bounds_->setLow(2, se_bounds_.minyaw);
    ompl_se_bounds_->setHigh(2, se_bounds_.maxyaw);
    state_space_ = std::make_shared<ompl::base::SE2StateSpace>();
    state_space_->as<ompl::base::SE2StateSpace>()->setBounds(*ompl_se_bounds_);
  } else {
    ompl_se_bounds_ = std::make_shared<ompl::base::RealVectorBounds>(3);
    ompl_se_bounds_->setLow(0, se_bounds_.minx);
    ompl_se_bounds_->setHigh(0, se_bounds_.maxx);
    ompl_se_bounds_->setLow(1, se_bounds_.miny);
    ompl_se_bounds_->setHigh(1, se_bounds_.maxy);
    ompl_se_bounds_->setLow(2, se_bounds_.minz);
    ompl_se_bounds_->setHigh(2, se_bounds_.maxz);
    state_space_ = std::make_shared<ompl::base::SE3StateSpace>();
    state_space_->as<ompl::base::SE3StateSpace>()->setBounds(*ompl_se_bounds_);
  }

  typedef std::shared_ptr<fcl::CollisionGeometry> CollisionGeometryPtr_t;
  CollisionGeometryPtr_t robot_body_box(
    new fcl::Box(
      robot_body_dimensions_.x,
      robot_body_dimensions_.y,
      robot_body_dimensions_.z));
  fcl::Transform3f tf2;
  fcl::CollisionObject robot_body_box_object(robot_body_box, tf2);
  robot_collision_object_ = std::make_shared<fcl::CollisionObject>(robot_body_box_object);
  octomap_subscriber_ = this->create_subscription<octomap_msgs::msg::Octomap>(
    octomap_topic_, rclcpp::SystemDefaultsQoS(),
    std::bind(&PlannerBenchMarking::octomapCallback, this, std::placeholders::_1));

  // Initialize pubs & subs
  plan_publisher_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>(
    sample_bencmark_plans_topic_.c_str(), rclcpp::SystemDefaultsQoS());
}

PlannerBenchMarking::~PlannerBenchMarking()
{

}

std::vector<ompl::geometric::PathGeometric> PlannerBenchMarking::doBenchMarking()
{
  /*Simple Setup*/
  ompl::geometric::SimpleSetup ss(state_space_);
  // define start & goal states
  if ((selected_state_space_ == "REEDS") || (selected_state_space_ == "DUBINS") ||
    (selected_state_space_ == "SE2"))
  {
    ompl::base::ScopedState<ompl::base::SE2StateSpace> start(state_space_), goal(state_space_);
    start->setXY(start_.x, start_.y);
    start->setYaw(start_.yaw);
    goal->setXY(goal_.x, goal_.y);
    goal->setYaw(goal_.yaw);
    ss.setStartAndGoalStates(start, goal, goal_tolerance_);
    ss.setStateValidityChecker(
      [this](const ompl::base::State * state)
      {
        return isStateValidSE2(state);
      });
  } else {
    ompl::base::ScopedState<ompl::base::SE3StateSpace> start(state_space_), goal(state_space_);
    start->setXYZ(start_.x, start_.y, start_.z);
    start->as<ompl::base::SO3StateSpace::StateType>(1)->setAxisAngle(0, 0, 1, start_.yaw);
    goal->setXYZ(goal_.x, goal_.y, goal_.z);
    goal->as<ompl::base::SO3StateSpace::StateType>(1)->setAxisAngle(0, 0, 1, goal_.yaw);
    ss.setStartAndGoalStates(start, goal, goal_tolerance_);
    ss.setStateValidityChecker(
      [this](const ompl::base::State * state)
      {
        return isStateValidSE3(state);
      });
  }

  auto si = ss.getSpaceInformation();
  ss.setOptimizationObjective(std::make_shared<ompl::base::PathLengthOptimizationObjective>(si));

  std::vector<ompl::geometric::PathGeometric> paths;
  // Create a sample plan for given problem with each planer in the benchmark
  if (publish_a_sample_bencmark_) {
    try {
      paths.push_back(makeAPlan(std::make_shared<ompl::geometric::PRMstar>(si), ss));
      ss.clear();
    } catch (const std::exception & e) {
      std::cerr << e.what() << '\n';
    }
    try {
      paths.push_back(makeAPlan(std::make_shared<ompl::geometric::LazyPRMstar>(si), ss));
      ss.clear();
    } catch (const std::exception & e) {
      std::cerr << e.what() << '\n';
    }
    try {
      paths.push_back(makeAPlan(std::make_shared<ompl::geometric::RRTstar>(si), ss));
      ss.clear();
    } catch (const std::exception & e) {
      std::cerr << e.what() << '\n';
    }
    try {
      paths.push_back(makeAPlan(std::make_shared<ompl::geometric::InformedRRTstar>(si), ss));
      ss.clear();
    } catch (const std::exception & e) {
      std::cerr << e.what() << '\n';
    }
    try {
      paths.push_back(makeAPlan(std::make_shared<ompl::geometric::SORRTstar>(si), ss));
      ss.clear();
    } catch (const std::exception & e) {
      std::cerr << e.what() << '\n';
    }
    try {
      paths.push_back(makeAPlan(std::make_shared<ompl::geometric::CForest>(si), ss));
      ss.clear();
    } catch (const std::exception & e) {
      std::cerr << e.what() << '\n';
    }
  }
  /*
  ompl::tools::Benchmark::Request request(planner_timeout_, max_memory_, num_benchmark_runs_);
  ompl::tools::Benchmark b(ss, "outdoor_plan_benchmarking");
  b.addPlanner(std::make_shared<ompl::geometric::PRMstar>(si));
  b.addPlanner(std::make_shared<ompl::geometric::LazyPRMstar>(si));
  b.addPlanner(std::make_shared<ompl::geometric::RRTstar>(si));
  b.addPlanner(std::make_shared<ompl::geometric::InformedRRTstar>(si));
  b.addPlanner(std::make_shared<ompl::geometric::SORRTstar>(si));
  b.addPlanner(std::make_shared<ompl::geometric::SPARStwo>(si));
  b.addPlanner(std::make_shared<ompl::geometric::CForest>(si));
  b.benchmark(request);
  b.saveResultsToFile(results_output_file_.c_str());*/

  return paths;
}

bool PlannerBenchMarking::isStateValidSE2(const ompl::base::State * state)
{
  if (is_octomap_ready_) {
    std::call_once(
      fcl_tree_from_octomap_once_, [this]() {
        std::shared_ptr<octomap::OcTree> octomap_octree =
        std::make_shared<octomap::OcTree>(octomap_voxel_size_);
        octomap_msgs::readTree<octomap::OcTree>(octomap_octree.get(), *octomap_msg_);
        fcl_octree_ = std::make_shared<fcl::OcTree>(octomap_octree);
        fcl_octree_collision_object_ = std::make_shared<fcl::CollisionObject>(
          std::shared_ptr<fcl::CollisionGeometry>(fcl_octree_));
        RCLCPP_INFO(
          this->get_logger(),
          "Recieved a valid Octomap, A FCL collision tree will be created from this "
          "octomap for state validity(aka collision check)");
      });
  } else {
    RCLCPP_ERROR(
      this->get_logger(),
      "The Octomap has not been recieved correctly, Collision check "
      "cannot be processed without a valid Octomap!");
    return false;
  }
  // cast the abstract state type to the type we expect
  const ompl::base::SE2StateSpace::StateType * se2_state =
    state->as<ompl::base::SE2StateSpace::StateType>();
  // check validity of state Fdefined by pos & rot
  fcl::Vec3f translation(se2_state->getX(), se2_state->getY(), 0.6);

  tf2::Quaternion myQuaternion;
  myQuaternion.setRPY(0, 0, se2_state->getYaw());
  fcl::Quaternion3f rotation(
    myQuaternion.getX(), myQuaternion.getY(),
    myQuaternion.getZ(), myQuaternion.getW());
  robot_collision_object_->setTransform(rotation, translation);
  fcl::CollisionRequest requestType(1, false, 1, false);
  fcl::CollisionResult collisionResult;
  fcl::collide(
    robot_collision_object_.get(),
    fcl_octree_collision_object_.get(), requestType, collisionResult);
  return !collisionResult.isCollision();
}

bool PlannerBenchMarking::isStateValidSE3(const ompl::base::State * state)
{
  if (is_octomap_ready_) {
    std::call_once(
      fcl_tree_from_octomap_once_, [this]() {
        std::shared_ptr<octomap::OcTree> octomap_octree =
        std::make_shared<octomap::OcTree>(octomap_voxel_size_);
        octomap_msgs::readTree<octomap::OcTree>(octomap_octree.get(), *octomap_msg_);
        fcl_octree_ = std::make_shared<fcl::OcTree>(octomap_octree);
        fcl_octree_collision_object_ = std::make_shared<fcl::CollisionObject>(
          std::shared_ptr<fcl::CollisionGeometry>(fcl_octree_));
        RCLCPP_INFO(
          this->get_logger(),
          "Recieved a valid Octomap, A FCL collision tree will be created from this "
          "octomap for state validity(aka collision check)");
      });
  } else {
    RCLCPP_ERROR(
      this->get_logger(),
      "The Octomap has not been recieved correctly, Collision check "
      "cannot be processed without a valid Octomap!");
    return false;
  }
  // cast the abstract state type to the type we expect
  const ompl::base::SE3StateSpace::StateType * se3state =
    state->as<ompl::base::SE3StateSpace::StateType>();
  // extract the first component of the state and cast it to what we expect
  const ompl::base::RealVectorStateSpace::StateType * pos =
    se3state->as<ompl::base::RealVectorStateSpace::StateType>(0);
  // extract the second component of the state and cast it to what we expect
  const ompl::base::SO3StateSpace::StateType * rot =
    se3state->as<ompl::base::SO3StateSpace::StateType>(1);
  // check validity of state Fdefined by pos & rot
  fcl::Vec3f translation(pos->values[0], pos->values[1], pos->values[2]);
  fcl::Quaternion3f rotation(rot->w, rot->x, rot->y, rot->z);
  robot_collision_object_->setTransform(rotation, translation);
  fcl::CollisionRequest requestType(1, false, 1, false);
  fcl::CollisionResult collisionResult;
  fcl::collide(
    robot_collision_object_.get(),
    fcl_octree_collision_object_.get(), requestType, collisionResult);
  return !collisionResult.isCollision();
}

void PlannerBenchMarking::octomapCallback(
  const octomap_msgs::msg::Octomap::ConstSharedPtr msg)
{
  const std::lock_guard<std::mutex> lock(octomap_mutex_);
  if (!is_octomap_ready_) {
    is_octomap_ready_ = true;
    octomap_msg_ = msg;
  }
}

ompl::geometric::PathGeometric PlannerBenchMarking::makeAPlan(
  const ompl::base::PlannerPtr & planner,
  ompl::geometric::SimpleSetup & ss)
{
  ss.setPlanner(planner);
  // attempt to solve the problem within one second of planning time
  ompl::base::PlannerStatus solved = ss.solve(planner_timeout_);
  ompl::geometric::PathGeometric original_path = ss.getSolutionPath();
  ompl::geometric::PathGeometric copy_path = ss.getSolutionPath();

  // Path smoothing using bspline
  ompl::geometric::PathSimplifier path_simlifier(ss.getSpaceInformation());
  if (path_simlifier.simplify(copy_path, 1.0, true)) {
    path_simlifier.smoothBSpline(copy_path, 1, 0.005);
    original_path = copy_path;
  }
  original_path.interpolate(interpolation_parameter_);
  return original_path;
}

void PlannerBenchMarking::publishSamplePlans(
  std::vector<ompl::geometric::PathGeometric> sample_paths)
{
  visualization_msgs::msg::MarkerArray marker_array;
  int path_index = 0;
  int total_poses = 0;
  for (auto && sample_path : sample_paths) {
    for (std::size_t curr_path_state = 0; curr_path_state < sample_path.getStateCount();
      curr_path_state++)
    {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = rclcpp::Clock().now();
      marker.type = visualization_msgs::msg::Marker::ARROW;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.lifetime = rclcpp::Duration::from_seconds(0);
      marker.scale.x = 0.4;
      marker.scale.y = 0.2;
      marker.scale.z = 0.2;
      marker.id = total_poses;
      marker.color = getColorByIndex(path_index);
      marker.ns = "path" + std::to_string(path_index);
      if (selected_state_space_ == "SE3") {
        auto se3_state =
          sample_path.getState(curr_path_state)->as<ompl::base::SE3StateSpace::StateType>();
        geometry_msgs::msg::Point p;
        p.x = se3_state->getX();
        p.y = se3_state->getY();
        p.z = se3_state->getZ();
        marker.pose.position = p;
        marker.pose.orientation.x = se3_state->rotation().x;
        marker.pose.orientation.y = se3_state->rotation().y;
        marker.pose.orientation.z = se3_state->rotation().z;
        marker.pose.orientation.w = se3_state->rotation().w;
      } else {
        auto se2_state =
          sample_path.getState(curr_path_state)->as<ompl::base::SE2StateSpace::StateType>();
        geometry_msgs::msg::Point p;
        p.x = se2_state->getX();
        p.y = se2_state->getY();
        p.z = 0.2;
        marker.pose.position = p;
        marker.pose.orientation =
          botanbot_utilities::getMsgQuaternionfromRPY(0, 0, se2_state->getYaw());
      }
      marker_array.markers.push_back(marker);
      total_poses++;
    }
    path_index++;
  }
  plan_publisher_->publish(marker_array);
}

std_msgs::msg::ColorRGBA PlannerBenchMarking::getColorByIndex(int index)
{
  std_msgs::msg::ColorRGBA result;
  switch (index) {
    case 0: // RED
      result.r = 0.8;
      result.g = 0.1;
      result.b = 0.1;
      result.a = 1.0;
      break;
    case 1: //GREEN
      result.r = 0.1;
      result.g = 0.8;
      result.b = 0.1;
      result.a = 1.0;
      break;
    case 2: //BLUE
      result.r = 0.0;
      result.g = 0.0;
      result.b = 1.0;
      result.a = 1.0;
      break;
    case 3: //WHITE
      result.r = 1.0;
      result.g = 1.0;
      result.b = 1.0;
      result.a = 1.0;
      break;
    case 4: //YELLOW
      result.r = 1.0;
      result.g = 1.0;
      result.b = 0.0;
      result.a = 1.0;
      break;
    case 5: //MAGENTA
      result.r = 1.0;
      result.g = 0.0;
      result.b = 1.0;
      result.a = 1.0;
      break;
    case 6: //LIME_GREEN
      result.r = 0.6;
      result.g = 1.0;
      result.b = 0.2;
      result.a = 1.0;
      break;
    case 7: //PINK
      result.r = 1.0;
      result.g = 0.4;
      result.b = 1;
      result.a = 1.0;
      break;
    case 8: //PURPLE
      result.r = 0.597;
      result.g = 0.0;
      result.b = 0.597;
      result.a = 1.0;
      break;
    case 9: //CYAN
      result.r = 0.0;
      result.g = 1.0;
      result.b = 1.0;
      result.a = 1.0;
      break;
  }
  return result;
}
}  // namespace botanbot_utilities

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<botanbot_utilities::PlannerBenchMarking>();
  while (rclcpp::ok() && !node->is_octomap_ready_) {
    rclcpp::spin_some(node->get_node_base_interface());
    RCLCPP_INFO(
      node->get_logger(),
      "Waiting for octomap to be ready In order to run planner bencmarking... ");
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
  RCLCPP_INFO(
    node->get_logger()
    ,
    "Octomap ready, running bencmark with given configurations");
  auto paths = node->doBenchMarking();
  while (rclcpp::ok()) {
    node->publishSamplePlans(paths);
    rclcpp::spin_some(node->get_node_base_interface());
    RCLCPP_INFO(
      node->get_logger(), "publishing planner bencmarking... CTRL +X to stop");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  RCLCPP_INFO(
    node->get_logger()
    ,
    "Benchmarking done , exiting successfully");
  rclcpp::shutdown();
  return 0;
}