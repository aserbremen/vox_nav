#include "botanbot_ompl_experimental/botanbot_ompl_experimental.hpp"


namespace botanbot_ompl_experimental
{

BotanbotOMPLExperimental::BotanbotOMPLExperimental()
: Node("botanbot_ompl_experimental_rclcpp_node"),
  robot_collision_geometry_(new fcl::Box(0.1, 0.1, 0.1))
{
  robot_collision_object_ = std::make_shared<fcl::CollisionObject>(robot_collision_geometry_);

  vis_pub_ =
    this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
  traj_pub_ =
    this->create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>("waypoints", 10);

  /*const std::string filename = "/home/ros2-foxy/fr_campus.bt";

  octomap::OcTree oc_tree_storage(0.1);
  oc_tree_storage.readBinary(filename);

  fcl_octree_ = std::make_shared<fcl::OcTree>(
    std::shared_ptr<const octomap::OcTree>(&oc_tree_storage));

  fcl_octree_collision_object_ =
    std::make_shared<fcl::CollisionObject>(std::shared_ptr<fcl::CollisionGeometry>(fcl_octree_));*/

  std::cout << "hi" << std::endl;
  const std::string filename = "/home/ros2-foxy/minelike.bt";

  octomap::OcTree temp_tree(0.1);
  temp_tree.readBinary(filename);
  fcl::OcTree * tree = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(&temp_tree));

  std::shared_ptr<fcl::CollisionObject> temp = std::make_shared<fcl::CollisionObject>(
    std::shared_ptr<fcl::CollisionGeometry>(
      tree));
  fcl_octree_collision_object_ = temp;

  plan();
}

BotanbotOMPLExperimental::~BotanbotOMPLExperimental()
{

}

bool BotanbotOMPLExperimental::isStateValid(const ompl::base::State * state)
{
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

class ChildOptimizationObjective : public ompl::base::OptimizationObjective
{
public:
  ChildOptimizationObjective(const ompl::base::SpaceInformationPtr & si)
  : ompl::base::OptimizationObjective(si)
  {
  }
  ompl::base::Cost motionCost(
    const ompl::base::State * s1,
    const ompl::base::State * s2) const override
  {
    ompl::base::Cost C1(stateCost(s1));
    ompl::base::Cost C2(stateCost(s2));
    return ompl::base::Cost(C1.value() + C2.value());
  }

  ompl::base::Cost stateCost(const ompl::base::State * s) const
  {
    const ompl::base::SE3StateSpace::StateType * Cstate3D =
      s->as<ompl::base::SE3StateSpace::StateType>();
    const ompl::base::RealVectorStateSpace::StateType * state3D =
      Cstate3D->as<ompl::base::RealVectorStateSpace::StateType>(0);
    double z = state3D->values[2];
    double y = state3D->values[1];
    return ompl::base::Cost(100 * z);
  }
};

ompl::base::OptimizationObjectivePtr BotanbotOMPLExperimental::get2(
  const ompl::base::SpaceInformationPtr & si)
{
  ompl::base::OptimizationObjectivePtr obj = std::make_shared<ChildOptimizationObjective>(si);
  obj->setCostToGoHeuristic(&ompl::base::goalRegionCostToGo);
  return obj;
}

void BotanbotOMPLExperimental::plan()
{
  // construct the state space we are planning in
  ompl::base::StateSpacePtr space(new ompl::base::SE3StateSpace());

  // set the bounds for the R^3 part of SE(3)
  ompl::base::RealVectorBounds bounds(3);

  bounds.setLow(0, -10);
  bounds.setHigh(0, 10);
  bounds.setLow(1, -10);
  bounds.setHigh(1, 10);
  bounds.setLow(2, 0);
  bounds.setHigh(2, 18);

  space->as<ompl::base::SE3StateSpace>()->setBounds(bounds);

  // construct an instance of  space information from this state space
  ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(space));

  // set state validity checking for this space
  si->setStateValidityChecker(
    std::bind(
      &BotanbotOMPLExperimental::isStateValid, this,
      std::placeholders::_1));

  // create a random start state
  ompl::base::ScopedState<ompl::base::SE3StateSpace> start(space);

  start->setXYZ(0.0, 0.0, 1); // reverse minelike

  start->as<ompl::base::SO3StateSpace::StateType>(1)->setIdentity();

  // create a random goal state
  ompl::base::ScopedState<ompl::base::SE3StateSpace> goal(space);
  goal->setXYZ(5, 5, 1); // reverse minelike
  goal->as<ompl::base::SO3StateSpace::StateType>(1)->setIdentity();

  // create a problem instance
  ompl::base::ProblemDefinitionPtr pdef(new ompl::base::ProblemDefinition(si));

  // set the start and goal states
  pdef->setStartAndGoalStates(start, goal);

  pdef->setOptimizationObjective(get2(si));

  // create a planner for the defined space
  //ompl::base::PlannerPtr planner(new ompl::geometric::RRTConnect(si));
  ompl::base::PlannerPtr planner(new ompl::geometric::RRTstar(si));
  //ompl::base::PlannerPtr planner(new ompl::geometric::PRMstar(si));

  // set the problem we are trying to solve for the planner
  planner->setProblemDefinition(pdef);

  // perform setup steps for the planner
  planner->setup();

  // print the settings for this space
  si->printSettings(std::cout);

  // print the problem settings
  pdef->print(std::cout);

  // attempt to solve the problem within one second of planning time
  ompl::base::PlannerStatus solved = planner->solve(20.0);

  std::cout << "Reached: " << std::endl;
  if (solved) {
    // get the goal representation from the problem definition (not the same as the goal state)
    // and inquire about the found path
    std::cout << "Found solution:" << std::endl;
    ompl::base::PathPtr path = pdef->getSolutionPath();
    ompl::geometric::PathGeometric * pth =
      pdef->getSolutionPath()->as<ompl::geometric::PathGeometric>();
    pth->printAsMatrix(std::cout);
    // print the path to screen
    trajectory_msgs::msg::MultiDOFJointTrajectory msg;
    trajectory_msgs::msg::MultiDOFJointTrajectoryPoint point_msg;

    msg.header.stamp = rclcpp::Clock().now();
    msg.header.frame_id = "map";
    msg.joint_names.clear();
    msg.points.clear();
    msg.joint_names.push_back("base_link");

    for (std::size_t path_idx = 0; path_idx < pth->getStateCount(); path_idx++) {
      const ompl::base::SE3StateSpace::StateType * se3state =
        pth->getState(path_idx)->as<ompl::base::SE3StateSpace::StateType>();

      // extract the first component of the state and cast it to what we expect
      const ompl::base::RealVectorStateSpace::StateType * pos =
        se3state->as<ompl::base::RealVectorStateSpace::StateType>(0);

      // extract the second component of the state and cast it to what we expect
      const ompl::base::SO3StateSpace::StateType * rot =
        se3state->as<ompl::base::SO3StateSpace::StateType>(1);

      //point_msg.time_from_start.sec(rclcpp::Clock().now());

      point_msg.transforms.resize(1);

      point_msg.transforms[0].translation.x = pos->values[0];
      point_msg.transforms[0].translation.y = pos->values[1];
      point_msg.transforms[0].translation.z = pos->values[2];

      point_msg.transforms[0].rotation.x = rot->x;
      point_msg.transforms[0].rotation.y = rot->y;
      point_msg.transforms[0].rotation.z = rot->z;
      point_msg.transforms[0].rotation.w = rot->w;

      msg.points.push_back(point_msg);

    }
    traj_pub_->publish(msg);

    //Path smoothing using bspline

    ompl::geometric::PathSimplifier * pathBSpline = new ompl::geometric::PathSimplifier(si);
    ompl::geometric::PathGeometric path_smooth(
      dynamic_cast<const ompl::geometric::PathGeometric &>(*pdef->getSolutionPath()));
    pathBSpline->smoothBSpline(path_smooth, 3);
    std::cout << "Smoothed Path" << std::endl;
    path_smooth.print(std::cout);

    //Publish path as markers
    visualization_msgs::msg::Marker marker;
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    vis_pub_->publish(marker);

    for (std::size_t idx = 0; idx < path_smooth.getStateCount(); idx++) {
      // cast the abstract state type to the type we expect
      const ompl::base::SE3StateSpace::StateType * se3state =
        path_smooth.getState(idx)->as<ompl::base::SE3StateSpace::StateType>();

      // extract the first component of the state and cast it to what we expect
      const ompl::base::RealVectorStateSpace::StateType * pos =
        se3state->as<ompl::base::RealVectorStateSpace::StateType>(0);

      // extract the second component of the state and cast it to what we expect
      const ompl::base::SO3StateSpace::StateType * rot =
        se3state->as<ompl::base::SO3StateSpace::StateType>(1);

      //marker.header.frame_id = "world";
      marker.header.frame_id = "map";
      marker.header.stamp = rclcpp::Clock().now();
      marker.ns = "path";
      marker.id = idx;
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = pos->values[0];
      marker.pose.position.y = pos->values[1];
      marker.pose.position.z = pos->values[2];
      marker.pose.orientation.x = rot->x;
      marker.pose.orientation.y = rot->y;
      marker.pose.orientation.z = rot->z;
      marker.pose.orientation.w = rot->w;
      marker.scale.x = 0.5;
      marker.scale.y = 0.5;
      marker.scale.z = 0.5;
      marker.color.a = 1.0;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      vis_pub_->publish(marker);
      std::cout << "Published marker: " << idx << std::endl;
    }
  } else {
    std::cout << "No solution found" << std::endl;
  }
}

} // namespace botanbot_ompl_experimental

int main(int argc, char ** argv)
{
  std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

  rclcpp::init(argc, argv);

  auto node = std::make_shared
    <botanbot_ompl_experimental::BotanbotOMPLExperimental>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
