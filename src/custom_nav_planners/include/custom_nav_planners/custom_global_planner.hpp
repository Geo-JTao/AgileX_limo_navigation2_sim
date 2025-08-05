#ifndef CUSTOM_GLOBAL_PLANNER_HPP
#define CUSTOM_GLOBAL_PLANNER_HPP

#include "nav2_core/global_planner.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include <memory>
#include <vector>

namespace custom_nav_planners
{
    // 只在头文件中定义一次Node结构体
    struct Node {
        int x, y;
        double g_cost, h_cost, f_cost;
        Node* parent;
        
        Node(int x_, int y_) : x(x_), y(y_), g_cost(0), h_cost(0), f_cost(0), parent(nullptr) {}
    };
    
    // 只在头文件中定义一次CompareNode结构体
    struct CompareNode {
        bool operator()(Node* a, Node* b) {
            return a->f_cost > b->f_cost; // 小顶堆
        }
    };

    class CustomGlobalPlanner : public nav2_core::GlobalPlanner
    {
    public:
        CustomGlobalPlanner();
        ~CustomGlobalPlanner() override;

        void configure(
            const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
            std::string name,
            std::shared_ptr<tf2_ros::Buffer> tf,
            std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

        void activate() override;
        void deactivate() override;
        void cleanup() override;

        nav_msgs::msg::Path createPlan(
            const geometry_msgs::msg::PoseStamped & start,
            const geometry_msgs::msg::PoseStamped & goal) override;

    private:
        // 代价地图相关参数
        nav2_costmap_2d::Costmap2D* costmap_;
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
        std::shared_ptr<tf2_ros::Buffer> tf_;
        rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
        std::string name_;
        double resolution_;
        double origin_x_;
        double origin_y_;
        int width_;
        int height_;

        // A*算法参数
        double weight_heuristic_;
        bool allow_unknown_;
        double inscribed_radius_;

        // 辅助函数
        double calculateHeuristic(int x1, int y1, int x2, int y2);
        bool isNodeValid(int x, int y);
        void worldToGrid(double wx, double wy, int & gx, int & gy);
        void gridToWorld(int gx, int gy, double & wx, double & wy);
        std::vector<Node*> getNeighbors(Node* current);
        std::vector<geometry_msgs::msg::PoseStamped> smoothPath(
            const std::vector<geometry_msgs::msg::PoseStamped> & path);
    };
}  // namespace custom_nav_planners

#endif  // CUSTOM_GLOBAL_PLANNER_HPP