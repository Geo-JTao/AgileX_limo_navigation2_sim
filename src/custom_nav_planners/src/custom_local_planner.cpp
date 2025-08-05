#include "custom_nav_planners/custom_local_planner.hpp"
#include <nav2_core/exceptions.hpp>
#include <nav2_util/node_utils.hpp>
#include <nav2_util/geometry_utils.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <algorithm>
#include <memory>
#include <cmath>

using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;

namespace custom_nav_planners
{
// 辅助函数：找到容器中具有最小计算值的元素
template<typename Iter, typename Getter>
Iter min_by(Iter begin, Iter end, Getter getCompareVal)
{
  if (begin == end) return end;
  auto lowest = getCompareVal(*begin);
  Iter lowest_it = begin;
  for (Iter it = ++begin; it != end; ++it) {
    auto comp = getCompareVal(*it);
    if (comp < lowest) {
      lowest = comp;
      lowest_it = it;
    }
  }
  return lowest_it;
}

// 构造函数：初始化成员变量，避免默认构造私有对象
CustomLocalPlanner::CustomLocalPlanner()
  : logger_(rclcpp::get_logger("CustomLocalPlanner")),
    transform_tolerance_(rclcpp::Duration::from_seconds(0.1)),
    speed_limit_(0.0),
    speed_limit_is_percentage_(false)
{}

void CustomLocalPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = node_.lock();  // 从WeakPtr获取shared_ptr

  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  // 声明并获取参数（参数前缀与插件名称绑定）
  declare_parameter_if_not_declared(
    node, name_ + ".desired_linear_vel", rclcpp::ParameterValue(0.2));
  declare_parameter_if_not_declared(
    node, name_ + ".lookahead_dist", rclcpp::ParameterValue(0.4));
  declare_parameter_if_not_declared(
    node, name_ + ".max_angular_vel", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, name_ + ".transform_tolerance", rclcpp::ParameterValue(0.1));

  // 读取参数
  node->get_parameter(name_ + ".desired_linear_vel", desired_linear_vel_);
  node->get_parameter(name_ + ".lookahead_dist", lookahead_dist_);
  node->get_parameter(name_ + ".max_angular_vel", max_angular_vel_);
  double transform_tolerance;
  node->get_parameter(name_ + ".transform_tolerance", transform_tolerance);
  transform_tolerance_ = rclcpp::Duration::from_seconds(transform_tolerance);

  // 创建路径发布器
  global_pub_ = node->create_publisher<nav_msgs::msg::Path>(name_ + "/received_global_plan", 1);

  RCLCPP_INFO(logger_, "Configured CustomLocalPlanner: %s", name_.c_str());
}

void CustomLocalPlanner::activate()
{
  global_pub_->on_activate();
  RCLCPP_INFO(logger_, "Activated CustomLocalPlanner: %s", name_.c_str());
}

void CustomLocalPlanner::deactivate()
{
  global_pub_->on_deactivate();
  RCLCPP_INFO(logger_, "Deactivated CustomLocalPlanner: %s", name_.c_str());
}

void CustomLocalPlanner::cleanup()
{
  global_pub_.reset();
  RCLCPP_INFO(logger_, "Cleaned up CustomLocalPlanner: %s", name_.c_str());
}

void CustomLocalPlanner::setPlan(const nav_msgs::msg::Path & path)
{
  global_path_ = path;
  global_pub_->publish(path);  // 发布接收到的全局路径
  RCLCPP_INFO(logger_, "Received global path with %zu points", path.poses.size());
}

// 实现速度限制设置函数
void CustomLocalPlanner::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  speed_limit_ = speed_limit;
  speed_limit_is_percentage_ = percentage;
  RCLCPP_INFO(logger_, "Speed limit set to: %.2f (percentage: %s)", 
              speed_limit_, percentage ? "true" : "false");
}

geometry_msgs::msg::TwistStamped CustomLocalPlanner::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist &,
  nav2_core::GoalChecker *)
{
  // 将全局路径转换到机器人坐标系
  auto transformed_plan = transformGlobalPlan(pose);

  // 找到第一个距离大于前瞻距离的路径点
  auto goal_pose_it = std::find_if(
    transformed_plan.poses.begin(), transformed_plan.poses.end(),
    [&](const auto & ps) {
      return hypot(ps.pose.position.x, ps.pose.position.y) >= lookahead_dist_;
    });

  // 如果所有点都在范围内，则取最后一个点
  if (goal_pose_it == transformed_plan.poses.end()) {
    goal_pose_it = std::prev(transformed_plan.poses.end());
  }
  auto goal_pose = goal_pose_it->pose;

  double linear_vel, angular_vel;

  // 纯追踪算法核心逻辑
  if (goal_pose.position.x > 0) {  // 目标在机器人前方
    // 计算曲率：2y/(x²+y²)
    double curvature = 2.0 * goal_pose.position.y /
      (goal_pose.position.x * goal_pose.position.x + goal_pose.position.y * goal_pose.position.y);
    linear_vel = desired_linear_vel_;
    
    // 应用速度限制
    if (speed_limit_is_percentage_ && speed_limit_ > 0) {
      linear_vel *= speed_limit_ / 100.0;
    } else if (!speed_limit_is_percentage_ && speed_limit_ > 0) {
      linear_vel = std::min(linear_vel, speed_limit_);
    }
    
    angular_vel = linear_vel * curvature;  // 角速度 = 线速度 * 曲率
  } else {  // 目标在机器人后方，只旋转
    linear_vel = 0.0;
    angular_vel = max_angular_vel_;
  }

  // 限制角速度在最大范围内
  angular_vel = std::clamp(angular_vel, -max_angular_vel_, max_angular_vel_);

  // 构造速度指令
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = pose.header.frame_id;
  cmd_vel.header.stamp = clock_->now();
  cmd_vel.twist.linear.x = linear_vel;
  cmd_vel.twist.angular.z = angular_vel;

  RCLCPP_INFO(
    logger_, "Computed velocity: linear.x=%.2f, angular.z=%.2f",
    cmd_vel.twist.linear.x, cmd_vel.twist.angular.z);
  return cmd_vel;
}

nav_msgs::msg::Path CustomLocalPlanner::transformGlobalPlan(
  const geometry_msgs::msg::PoseStamped & pose)
{
  if (global_path_.poses.empty()) {
    throw nav2_core::PlannerException("Received empty global plan");
  }

  // 将机器人位姿转换到全局路径坐标系
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!transformPose(
      tf_, global_path_.header.frame_id, pose,
      robot_pose, transform_tolerance_))
  {
    throw nav2_core::PlannerException("Failed to transform robot pose to global plan frame");
  }

  // 只处理局部代价图范围内的路径点
  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  double dist_threshold = std::max(costmap->getSizeInCellsX(), costmap->getSizeInCellsY()) *
    costmap->getResolution() / 2.0;

  // 找到离机器人最近的路径点
  auto transformation_begin = min_by(
    global_path_.poses.begin(), global_path_.poses.end(),
    [&robot_pose](const geometry_msgs::msg::PoseStamped & ps) {
      return euclidean_distance(robot_pose, ps);
    });

  // 找到超出代价图范围的第一个点（后续点不处理）
  auto transformation_end = std::find_if(
    transformation_begin, global_path_.poses.end(),
    [&](const auto & global_plan_pose) {
      return euclidean_distance(robot_pose, global_plan_pose) > dist_threshold;
    });

  // 转换路径点到机器人坐标系
  nav_msgs::msg::Path transformed_plan;
  transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();
  transformed_plan.header.stamp = pose.header.stamp;

  std::transform(
    transformation_begin, transformation_end,
    std::back_inserter(transformed_plan.poses),
    [&](const auto & global_plan_pose) {
      geometry_msgs::msg::PoseStamped stamped_pose, transformed_pose;
      stamped_pose.header.frame_id = global_path_.header.frame_id;
      stamped_pose.header.stamp = pose.header.stamp;
      stamped_pose.pose = global_plan_pose.pose;
      transformPose(
        tf_, costmap_ros_->getBaseFrameID(),
        stamped_pose, transformed_pose, transform_tolerance_);
      return transformed_pose;
    });

  // 修剪已通过的路径点（优化后续计算）
  global_path_.poses.erase(global_path_.poses.begin(), transformation_begin);
  global_pub_->publish(transformed_plan);

  if (transformed_plan.poses.empty()) {
    throw nav2_core::PlannerException("Transformed plan has no valid poses");
  }

  return transformed_plan;
}

bool CustomLocalPlanner::transformPose(
  const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::string frame,
  const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose,
  const rclcpp::Duration & transform_tolerance
) const
{
  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    tf->transform(in_pose, out_pose, frame);
    return true;
  } catch (tf2::ExtrapolationException & ex) {
    auto transform = tf->lookupTransform(frame, in_pose.header.frame_id, tf2::TimePointZero);
    if ((rclcpp::Time(in_pose.header.stamp) - rclcpp::Time(transform.header.stamp)) > transform_tolerance) {
      RCLCPP_WARN(logger_, "Transform data too old: %s", ex.what());
      return false;
    } else {
      tf2::doTransform(in_pose, out_pose, transform);
      return true;
    }
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(logger_, "Transform failed: %s", ex.what());
    return false;
  }
}

}  // namespace custom_nav_planners

// 注册为Nav2插件
PLUGINLIB_EXPORT_CLASS(custom_nav_planners::CustomLocalPlanner, nav2_core::Controller)
