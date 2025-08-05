#include "custom_nav_planners/custom_global_planner.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <algorithm>
#include <unordered_map>
#include <queue>

namespace custom_nav_planners
{

CustomGlobalPlanner::CustomGlobalPlanner()
: costmap_(nullptr),
  resolution_(0.05),
  origin_x_(0.0),
  origin_y_(0.0),
  width_(0),
  height_(0),
  weight_heuristic_(1.0),
  allow_unknown_(false),
  inscribed_radius_(0.3)
{}

CustomGlobalPlanner::~CustomGlobalPlanner()
{
  cleanup();
}

void CustomGlobalPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();

  // 获取代价地图参数
  resolution_ = costmap_->getResolution();
  origin_x_ = costmap_->getOriginX();
  origin_y_ = costmap_->getOriginY();
  width_ = costmap_->getSizeInCellsX();  // 正确的API调用
  height_ = costmap_->getSizeInCellsY(); // 正确的API调用

  // 声明并获取参数
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".weight_heuristic", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".allow_unknown", rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".inscribed_radius", rclcpp::ParameterValue(0.3));

  node_->get_parameter(name_ + ".weight_heuristic", weight_heuristic_);
  node_->get_parameter(name_ + ".allow_unknown", allow_unknown_);
  node_->get_parameter(name_ + ".inscribed_radius", inscribed_radius_);

  RCLCPP_INFO(node_->get_logger(), "Configuring A* Global Planner: %s", name_.c_str());
  RCLCPP_INFO(node_->get_logger(), "Map resolution: %.3f m/cell", resolution_);
  RCLCPP_INFO(node_->get_logger(), "Map size: %dx%d cells", width_, height_);
}

void CustomGlobalPlanner::activate()
{
  RCLCPP_INFO(node_->get_logger(), "Activating A* Global Planner");
}

void CustomGlobalPlanner::deactivate()
{
  RCLCPP_INFO(node_->get_logger(), "Deactivating A* Global Planner");
}

void CustomGlobalPlanner::cleanup()
{
  RCLCPP_INFO(node_->get_logger(), "Cleaning up A* Global Planner");
}

double CustomGlobalPlanner::calculateHeuristic(int x1, int y1, int x2, int y2)
{
  // 欧几里得距离作为启发式函数
  double dx = (x1 - x2) * resolution_;
  double dy = (y1 - y2) * resolution_;
  return weight_heuristic_ * std::sqrt(dx * dx + dy * dy);
}

bool CustomGlobalPlanner::isNodeValid(int x, int y)
{
  // 检查是否在地图范围内
  if (x < 0 || x >= width_ || y < 0 || y >= height_) {
    return false;
  }

  // 检查代价是否为障碍物
  unsigned char cost = costmap_->getCost(x, y);
  
  // 致命障碍物或超出代价阈值的区域视为无效
  if (cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {
    return false;
  }
  
  // 如果不允许未知区域，检查是否为未知
  if (!allow_unknown_ && cost == nav2_costmap_2d::NO_INFORMATION) {
    return false;
  }

  return true;
}

void CustomGlobalPlanner::worldToGrid(double wx, double wy, int & gx, int & gy)
{
  gx = static_cast<int>((wx - origin_x_) / resolution_);
  gy = static_cast<int>((wy - origin_y_) / resolution_);
}

void CustomGlobalPlanner::gridToWorld(int gx, int gy, double & wx, double & wy)
{
  wx = origin_x_ + (gx + 0.5) * resolution_;
  wy = origin_y_ + (gy + 0.5) * resolution_;
}

std::vector<Node*> CustomGlobalPlanner::getNeighbors(Node* current)
{
  std::vector<Node*> neighbors;
  int x = current->x;
  int y = current->y;

  // 8个方向的邻居（包括对角线）
  int dirs[8][2] = {
    {1, 0}, {-1, 0}, {0, 1}, {0, -1},
    {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
  };

  for (auto & dir : dirs) {
    int nx = x + dir[0];
    int ny = y + dir[1];
    
    if (isNodeValid(nx, ny)) {
      // 对角线移动代价更高（√2倍）
      double move_cost = (dir[0] != 0 && dir[1] != 0) ? 
        std::sqrt(2) * resolution_ : resolution_;
      
      Node* neighbor = new Node(nx, ny);
      neighbor->g_cost = current->g_cost + move_cost;
      neighbor->parent = current;
      neighbors.push_back(neighbor);
    }
  }

  return neighbors;
}

std::vector<geometry_msgs::msg::PoseStamped> CustomGlobalPlanner::smoothPath(
  const std::vector<geometry_msgs::msg::PoseStamped> & path)
{
  if (path.size() < 3) {
    return path;
  }

  std::vector<geometry_msgs::msg::PoseStamped> smoothed_path = path;
  
  // 滑动平均滤波平滑路径
  const int window_size = 3;
  for (size_t i = window_size; i < smoothed_path.size() - window_size; ++i) {
    double sum_x = 0.0, sum_y = 0.0;
    
    for (int j = -window_size; j <= window_size; ++j) {
      sum_x += smoothed_path[i + j].pose.position.x;
      sum_y += smoothed_path[i + j].pose.position.y;
    }
    
    smoothed_path[i].pose.position.x = sum_x / (2 * window_size + 1);
    smoothed_path[i].pose.position.y = sum_y / (2 * window_size + 1);
  }

  return smoothed_path;
}

nav_msgs::msg::Path CustomGlobalPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = start.header.frame_id;
  path.header.stamp = node_->now();

  // 将起点和终点转换为网格坐标
  int start_x, start_y, goal_x, goal_y;
  worldToGrid(start.pose.position.x, start.pose.position.y, start_x, start_y);
  worldToGrid(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y);

  // 检查起点和终点是否有效
  if (!isNodeValid(start_x, start_y)) {
    RCLCPP_ERROR(node_->get_logger(), "Start position is invalid (obstacle or out of bounds)");
    path.poses.push_back(start);
    path.poses.push_back(goal);
    return path;
  }

  if (!isNodeValid(goal_x, goal_y)) {
    RCLCPP_ERROR(node_->get_logger(), "Goal position is invalid (obstacle or out of bounds)");
    path.poses.push_back(start);
    path.poses.push_back(goal);
    return path;
  }

  // 检查起点和终点是否相同
  if (start_x == goal_x && start_y == goal_y) {
    RCLCPP_INFO(node_->get_logger(), "Start and goal positions are the same");
    path.poses.push_back(start);
    return path;
  }

  // A*算法实现
  std::priority_queue<Node*, std::vector<Node*>, CompareNode> open_list;
  std::unordered_map<int, std::unordered_map<int, Node*>> all_nodes;

  // 初始化起点节点
  Node* start_node = new Node(start_x, start_y);
  start_node->g_cost = 0.0;
  start_node->h_cost = calculateHeuristic(start_x, start_y, goal_x, goal_y);
  start_node->f_cost = start_node->g_cost + start_node->h_cost;
  open_list.push(start_node);
  all_nodes[start_x][start_y] = start_node;

  bool found_goal = false;
  Node* current_node = nullptr;

  // 执行A*搜索
  while (!open_list.empty()) {
    // 获取f_cost最小的节点
    current_node = open_list.top();
    open_list.pop();

    // 检查是否到达目标
    if (current_node->x == goal_x && current_node->y == goal_y) {
      found_goal = true;
      break;
    }

    // 获取所有邻居节点
    std::vector<Node*> neighbors = getNeighbors(current_node);

    for (Node* neighbor : neighbors) {
      int x = neighbor->x;
      int y = neighbor->y;

      // 计算启发式代价
      neighbor->h_cost = calculateHeuristic(x, y, goal_x, goal_y);
      neighbor->f_cost = neighbor->g_cost + neighbor->h_cost;

      // 检查该节点是否已在地图中
      if (all_nodes.find(x) != all_nodes.end() && all_nodes[x].find(y) != all_nodes[x].end()) {
        Node* existing_node = all_nodes[x][y];
        if (neighbor->g_cost < existing_node->g_cost) {
          // 找到更优路径，更新节点信息
          existing_node->g_cost = neighbor->g_cost;
          existing_node->f_cost = neighbor->f_cost;
          existing_node->parent = current_node;
          open_list.push(existing_node);
        }
        delete neighbor;  // 释放临时节点
      } else {
        // 添加新节点到开放列表
        open_list.push(neighbor);
        all_nodes[x][y] = neighbor;
      }
    }
  }

  // 回溯路径
  if (found_goal && current_node != nullptr) {
    std::vector<Node*> path_nodes;
    
    // 从目标节点回溯到起点
    while (current_node != nullptr) {
      path_nodes.push_back(current_node);
      current_node = current_node->parent;
    }
    
    // 反转路径，从起点到目标
    std::reverse(path_nodes.begin(), path_nodes.end());

    // 转换为世界坐标并添加到路径消息
    for (Node* node : path_nodes) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      
      // 转换网格坐标到世界坐标
      gridToWorld(node->x, node->y, 
                 pose.pose.position.x, pose.pose.position.y);
      pose.pose.position.z = start.pose.position.z;

      // 计算方向角（朝向路径下一个点）
      auto it = std::find(path_nodes.begin(), path_nodes.end(), node);
      if (it != path_nodes.end() && std::next(it) != path_nodes.end()) {
        Node* next_node = *std::next(it);
        double dx = (next_node->x - node->x) * resolution_;
        double dy = (next_node->y - node->y) * resolution_;
        double yaw = std::atan2(dy, dx);
        
        tf2::Quaternion quat;
        quat.setRPY(0, 0, yaw);
        pose.pose.orientation.x = quat.x();
        pose.pose.orientation.y = quat.y();
        pose.pose.orientation.z = quat.z();
        pose.pose.orientation.w = quat.w();
      } else {
        // 最后一个点使用目标的朝向
        pose.pose.orientation = goal.pose.orientation;
      }
      
      path.poses.push_back(pose);
    }

    RCLCPP_INFO(node_->get_logger(), "A* found path with %zu points", path.poses.size());
    
    // 平滑路径
    path.poses = smoothPath(path.poses);
  } else {
    RCLCPP_WARN(node_->get_logger(), "A* could not find a valid path, using direct path");
    path.poses.push_back(start);
    path.poses.push_back(goal);
  }

  // 清理所有节点内存
  for (auto & x_entry : all_nodes) {
    for (auto & y_entry : x_entry.second) {
      delete y_entry.second;
    }
  }

  return path;
}

}  // namespace custom_nav_planners

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(custom_nav_planners::CustomGlobalPlanner, nav2_core::GlobalPlanner)
