#ifndef CUSTOM_GLOBAL_PLANNER_HPP_
#define CUSTOM_GLOBAL_PLANNER_HPP_
 
#include <nav2_core/global_planner.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
 
namespace custom_nav_planners
{
class CustomGlobalPlanner : public nav2_core::GlobalPlanner
{
public:
  CustomGlobalPlanner();
  void configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
      std::string name,
      std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void activate() override;
  void deactivate() override;
  void cleanup() override;
  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped & start,
                                 const geometry_msgs::msg::PoseStamped & goal) override;
 
private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::string name_;
};
}  // namespace custom_nav_planners
 
#endif  // CUSTOM_GLOBAL_PLANNER_HPP_