#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

// Create a ROS logger
auto const logger = rclcpp::get_logger("ur5_moveit");

// Create pose goble variable
geometry_msgs::msg::Pose current_pose;
geometry_msgs::msg::Pose target_pose;


bool service_call = false;
bool move_fail = false;

void get_current_pose(moveit::planning_interface::MoveGroupInterface &move_group_interface) {
  // Get current pose
  current_pose = move_group_interface.getCurrentPose().pose;

  // Set Target pose
  target_pose.position.x = 0.5;
  target_pose.position.y = 0.0;
  target_pose.position.z = 0.3;
  target_pose.orientation.x = current_pose.orientation.x;
  target_pose.orientation.y = current_pose.orientation.y;
  target_pose.orientation.z = current_pose.orientation.z;
  target_pose.orientation.w = current_pose.orientation.w;

  // Print the current pose
  RCLCPP_INFO(logger,
    "Current position\npose:\n\tX:%f\n\tY:%f\n\tZ:%f\norientation:\n\tx:%f\n\ty:%f\n\tz:%f\n\tw:%f", 
    current_pose.position.x,
    current_pose.position.y,
    current_pose.position.z,
    current_pose.orientation.x,
    current_pose.orientation.y,
    current_pose.orientation.z,
    current_pose.orientation.w);
}

void move_ur5(moveit::planning_interface::MoveGroupInterface &move_group_interface) {
  move_group_interface.setPoseTarget(target_pose);

  // Plan a Cartesian path
  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(current_pose);  // Start from the current pose
  waypoints.push_back(target_pose);  // Move to the target pose

  moveit_msgs::msg::RobotTrajectory trajectory;
  const double eef_step = 0.01;
  double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, trajectory);

  bool success = (fraction > 0.95);  // Consider successful if more than 95% of the path is planned
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.trajectory = trajectory;

  // Execute the plan
  if(success) {
    RCLCPP_INFO(logger, "Planing success!");
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planing failed!");
    move_fail = true;
    return;
  }
      
  RCLCPP_INFO(logger, "Move Done!");
}

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "ur5_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // We spin up a SingleThreadedExecutor so MoveItVisualTools interact with ROS
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");

  // Log that the service is ready
  RCLCPP_INFO(logger, "UR5 MoveIt Node has been started");

  get_current_pose(move_group_interface);
  move_ur5(move_group_interface);

  // Keep the node running indefinitely by waiting for the executor thread
  spinner.join();

  // Shutdown ROS (this part won't be reached unless node is stopped)
  rclcpp::shutdown();
  return 0;
}


