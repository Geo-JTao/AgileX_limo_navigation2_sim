#include "custom_nav_planners/custom_global_planner.hpp"
 
namespace custom_nav_planners
{
CustomGlobalPlanner::CustomGlobalPlanner() {}
 
void CustomGlobalPlanner::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  costmap_ros_ = costmap_ros;
  RCLCPP_INFO(node_->get_logger(), "Configuring CustomGlobalPlanner: %s", name_.c_str());
}
 
void CustomGlobalPlanner::activate()
{
  RCLCPP_INFO(node_->get_logger(), "Activating CustomGlobalPlanner");
}
 
void CustomGlobalPlanner::deactivate()
{
  RCLCPP_INFO(node_->get_logger(), "Deactivating CustomGlobalPlanner");
}
 
void CustomGlobalPlanner::cleanup()
{
  RCLCPP_INFO(node_->get_logger(), "Cleaning up CustomGlobalPlanner");
}
 
nav_msgs::msg::Path CustomGlobalPlanner::createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = start.header.frame_id;
  path.header.stamp = node_->now();
 
  // Simple straight-line path (for demonstration)
  geometry_msgs::msg::PoseStamped pose;
  pose.header = path.header;
  double dx = (goal.pose.position.x - start.pose.position.x) / 10.0;
  double dy = (goal.pose.position.y - start.pose.position.y) / 10.0;
 
  for (int i = 0; i <= 10; ++i) {
    pose.pose.position.x = start.pose.position.x + i * dx;
    pose.pose.position.y = start.pose.position.y + i * dy;
    pose.pose.orientation = goal.pose.orientation;
    path.poses.push_back(pose);
  }
 
  RCLCPP_INFO(node_->get_logger(), "Generated path with %zu points", path.poses.size());
  return path;
}
}  // namespace custom_nav_planners
 
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(custom_nav_planners::CustomGlobalPlanner, nav2_core::GlobalPlanner)