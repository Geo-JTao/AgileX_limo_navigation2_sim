#ifndef CUSTOM_LOCAL_PLANNER_HPP_
#define CUSTOM_LOCAL_PLANNER_HPP_

#include <nav2_core/controller.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <tf2_ros/buffer.h>
#include <memory>
#include <string>

namespace custom_nav_planners
{
class CustomLocalPlanner : public nav2_core::Controller
{
public:
  CustomLocalPlanner();
  ~CustomLocalPlanner() override = default;

  // 修正configure函数参数，使用WeakPtr匹配基类
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void activate() override;
  void deactivate() override;
  void cleanup() override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  // 实现缺失的纯虚函数
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  // 辅助函数：将全局路径转换到机器人坐标系
  nav_msgs::msg::Path transformGlobalPlan(const geometry_msgs::msg::PoseStamped & pose);

  // 辅助函数：坐标变换
  bool transformPose(
    const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::string frame,
    const geometry_msgs::msg::PoseStamped & in_pose,
    geometry_msgs::msg::PoseStamped & out_pose,
    const rclcpp::Duration & transform_tolerance
  ) const;

  // 节点与工具类
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;  // 改为WeakPtr匹配基类
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::string name_;
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;

  // 纯追踪算法参数
  double desired_linear_vel_;    // 期望线速度
  double lookahead_dist_;        // 前瞻距离
  double max_angular_vel_;       // 最大角速度
  rclcpp::Duration transform_tolerance_;  // 坐标变换超时时间
  double speed_limit_;           // 速度限制
  bool speed_limit_is_percentage_; // 速度限制是否为百分比

  // 路径与发布器
  nav_msgs::msg::Path global_path_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> global_pub_;
};
}  // namespace custom_nav_planners

#endif  // CUSTOM_LOCAL_PLANNER_HPP_
