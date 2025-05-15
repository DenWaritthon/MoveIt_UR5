#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include "ur5_interfaces/srv/move_it_target.hpp"
#include "ur5_interfaces/srv/move_it_report.hpp"


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

void set_target(const std::shared_ptr<ur5_interfaces::srv::MoveItTarget::Request> request,
  std::shared_ptr<ur5_interfaces::srv::MoveItTarget::Response> response)
{                             
  // Get pick target from request
  target_pose.position.x = request->target.position.x;
  target_pose.position.y = request->target.position.y;
  target_pose.position.z = request->target.position.z;

  // Print the pick target pose
  RCLCPP_INFO(logger, "Target\npose:\n\tX:%f\n\tY:%f\n\tZ:%f",
    target_pose.position.x,
    target_pose.position.y,
    target_pose.position.z);

  RCLCPP_INFO(logger, "Set target success!");
  service_call = true;
  response->success = true;
}

void move_ur5(moveit::planning_interface::MoveGroupInterface &move_group_interface) {
  // Set the target orientation
  target_pose.orientation.x = current_pose.orientation.x;
  target_pose.orientation.y = current_pose.orientation.y;
  target_pose.orientation.z = current_pose.orientation.z;
  target_pose.orientation.w = current_pose.orientation.w;
  
  // Set the target pose
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
    RCLCPP_INFO(logger, "Move Done!");
    move_fail = false;
  } else {
    RCLCPP_ERROR(logger, "Planing failed!");
    move_fail = true;
    return;
  }
      
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

  // Set service server
  auto service = node->create_service<ur5_interfaces::srv::MoveItTarget>("/target", &set_target);

  // Create a service client
  auto report = node->create_client<ur5_interfaces::srv::MoveItReport>("/ur5_report");

  // Wait for the service to be available
  while (!report->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(logger, "Interrupted while waiting for the service '/ur5_report'. Exiting...");
      return 0;
    }
    RCLCPP_INFO(logger, "Service '/ur5_report' not available, waiting again...");
  }

  // Log that the service is ready
  RCLCPP_INFO(logger, "Service client for '/ur5_report' is ready");
  RCLCPP_INFO(logger, "Service '/target' is ready");
  RCLCPP_INFO(logger, "UR5 MoveIt Node has been started");

  while (rclcpp::ok()) {
    // Wait for the service to be called
    if (service_call) {
      // Get current pose
      get_current_pose(move_group_interface);

      // Move the UR5 to the target pose
      move_ur5(move_group_interface);

      if (move_fail) {
        RCLCPP_ERROR(logger, "Move step failed!");
      } else {
        RCLCPP_INFO(logger, "Move step success!");

        // Create a request to send to the service
        auto request = std::make_shared<ur5_interfaces::srv::MoveItReport::Request>();
        get_current_pose(move_group_interface);
        request->move_done = true;
        request->current_pose = current_pose;
        // Call the service and wait for the result
        auto result = report->async_send_request(request);
      }

      // Reset the service call flag
      service_call = false;
    }
  }

  // Keep the node running indefinitely by waiting for the executor thread
  spinner.join();

  // Shutdown ROS (this part won't be reached unless node is stopped)
  rclcpp::shutdown();
  return 0;
}


