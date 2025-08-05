#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from visualization_msgs.msg import MarkerArray, Marker
from copy import deepcopy
import numpy as np
import tf_transformations

class AutoNavigator(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.navigator = BasicNavigator()
        # 发布导航目标点在rviz可视化
        self.marker_pub = self.create_publisher(MarkerArray, 'waypoint_markers', 10)
        # 订阅amcl_pose获取当前位置
        self.current_pose = None
        self.create_subscription(
            PoseWithCovarianceStamped, 
            '/amcl_pose', 
            self.amcl_pose_callback, 
            10
        )
        # 等待初始位置
        self.get_logger().info("等待初始位置...")
        while self.current_pose is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # 设置初始位置
        initial_pose = self.set_initial_pose(x=0.0, y=0.0, theta=0.0)
        # 等待Nav2系统完全激活
        self.get_logger().info("等待Nav2系统激活...")
        self.navigator.waitUntilNav2Active()
        self.get_logger().info("Nav2已激活！")
        self.cleanup_markers()
    
    def amcl_pose_callback(self, msg):
        """保存当前机器人位置"""
        self.current_pose = msg.pose.pose
        # self.get_logger().info(f"当前位置: ({self.current_pose.position.x:.2f}, {self.current_pose.position.y:.2f})", throttle_duration_sec=1.0)

    def get_current_pose_stamped(self):
        """获取当前位姿的PoseStamped消息"""
        if self.current_pose is None:
            self.get_logger().warn("当前位置不可用，使用默认位置")
            return self._create_pose_stamped(0.0, 0.0, 0.0)
        
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'map'
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose = deepcopy(self.current_pose)
        return pose_stamped

    def set_initial_pose(self, x=0.0, y=0.0, theta=0.0):
        """设置初始位置（代替RViz的2D Pose Estimate）只需要在nav2算法启动时操作一次"""
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = x
        initial_pose.pose.position.y = y
        # 将欧拉角转换为四元数（theta为弧度）
        initial_pose.pose.orientation.z = np.sin(theta/2)
        initial_pose.pose.orientation.w = np.cos(theta/2)
        self.navigator.setInitialPose(initial_pose)
        self.get_logger().info(f"初始位置设置为: ([{x}, {y}], theta = {theta})")
        return initial_pose

    def publish_waypoint_markers(self, poses):
        """发布路点可视化标记,参数:poses: PoseStamped列表,要显示的路点位置"""
        marker_array = MarkerArray()
        for i, pose in enumerate(poses):
            marker = Marker()
            marker.header = pose.header
            marker.ns = "waypoints"
            marker.id = i
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose = pose.pose
            # 设置箭头大小
            marker.scale.x = 0.15  # 箭头长度
            marker.scale.y = 0.05  # 箭头宽度
            marker.scale.z = 0.05  # 箭头高度
            # 设置颜色（透明度/rgb）
            marker.color.a = 1.0 
            marker.color.r = 1.0  
            marker.color.g = 0.0
            marker.color.b = 0.0
            # 设置生命周期 (0表示永久)
            marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg()
            marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)
        self.get_logger().info(f"已发布 {len(poses)} 个路点标记")

    def cleanup_markers(self):
        """清除所有路点标记（显式删除）"""
        marker_array = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "map"  # 必须和发布时一致
        marker.ns = "waypoints"       # 必须和发布时一致
        marker.action = Marker.DELETEALL  # 删除所有同ns下的标记
        marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)
        self.get_logger().info("已清除所有路点标记")

    def waypoints_to_pose_sequence(self, waypoints):
        # 将[(x1,y1,theta1), (x2,y2,theta2), ...] 路点转换为PoseStamped列表
        pose_sequence = []
        for i in range(len(waypoints)):
            x, y, theta = waypoints[i]
            pose = self._create_pose_stamped(x, y, theta)
            pose_sequence.append(pose)
        return pose_sequence

    def _create_pose_stamped(self, x, y, theta):
        """创建PoseStamped消息，支持不指定方向(theta=None)"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        
        if theta is not None:
            pose.pose.orientation.z = np.sin(theta / 2)
            pose.pose.orientation.w = np.cos(theta / 2)
        else:
            # 如果不指定方向，使用中性方向 (0度)
            pose.pose.orientation.w = 1.0
        
        return pose

    def _normalize_angle(self, angle):
        """将角度归一化到[-π, π]范围内"""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle
    
    def waypoints_to_pose_sequence_with_interpolation(self, waypoints, interpolation_step=0.25):
        """将路点列表转换为PoseStamped序列，并插入中间点"""
        pose_sequence = []
        for i in range(len(waypoints)):
            x, y, theta = waypoints[i]
            pose = self._create_pose_stamped(x, y, theta)
            pose_sequence.append(pose)
            # 如果不是最后一个点，插入中间点（位置和角度均插值）
            if i < len(waypoints) - 1:
                next_x, next_y, next_theta = waypoints[i + 1]
                distance = np.sqrt((next_x - x) ** 2 + (next_y - y) ** 2)
                num_points = int(distance / interpolation_step)
                # 角度插值（处理角度环绕问题）
                delta_theta = self._normalize_angle(next_theta - theta)
                for j in range(1, num_points):
                    ratio = j / num_points
                    inter_x = x + ratio * (next_x - x)
                    inter_y = y + ratio * (next_y - y)
                    inter_theta = theta + ratio * delta_theta
                    # 创建中间位姿
                    inter_pose = self._create_pose_stamped(inter_x, inter_y, inter_theta)
                    pose_sequence.append(inter_pose)
        return pose_sequence

    def navigate_to_pose(self, pose, timeout=600.0):
        """导航到某个指定位置"""
        self.get_logger().info(f"开始导航到目标: ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})")
        self.navigator.goToPose(pose)
        start_time = self.get_clock().now()
        # 等待导航到目标点，打印进度信息
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            current_time = self.get_clock().now()
            # 将纳秒转换为秒
            elapsed_time = (current_time - start_time).nanoseconds / 1e9
            if int(elapsed_time) % 1 == 0: 
                self.get_logger().info(
                    f"剩余距离: {feedback.distance_remaining:.2f}米, "
                    f"已耗时: {elapsed_time:.1f}秒",
                    throttle_duration_sec=1  # 每秒最多打印一次
                )
            # 超时检查
            if elapsed_time > timeout:
                self.get_logger().warn("导航超时，取消任务！")
                self.navigator.cancelTask()
                return TaskResult.FAILED
        # 单个目标位置导航结果检查
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("导航成功！")
        elif result == TaskResult.CANCELED:
            self.get_logger().warn("导航被取消！")
        elif result == TaskResult.FAILED:
            self.get_logger().error("导航失败！")
        
        return result

    def spin_in_place(self, target_theta, timeout=30.0):
        """在原地旋转到指定角度"""
        # 获取当前位置
        current_pose_stamped = self.get_current_pose_stamped()
        
        # 创建旋转目标位姿（位置不变，只改变方向）
        spin_pose = deepcopy(current_pose_stamped)
        spin_pose.pose.orientation.z = np.sin(target_theta / 2)
        spin_pose.pose.orientation.w = np.cos(target_theta / 2)
        
        self.get_logger().info(f"开始原地旋转至: {target_theta:.2f} 弧度")
        return self.navigate_to_pose(spin_pose, timeout)

    def run_navigation_cirle(self, center_x, center_y, radius):
        self.get_logger().info("开始圆形路径导航...")
        # 生成圆形路径点
        waypoints = []
        for angle in np.linspace(0, 2*np.pi, 16):  # 16个点构成圆
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = center_x + radius * np.cos(angle)
            pose.pose.position.y = center_y + radius * np.sin(angle)
            # 设置朝向切线方向
            tangent_angle = angle + np.pi/2  # 切线方向=法线方向+90度
            pose.pose.orientation.z = np.sin(tangent_angle/2)
            pose.pose.orientation.w = np.cos(tangent_angle/2)
            waypoints.append(pose)
        self.publish_waypoint_markers(waypoints)
        # 连续导航圆形路径
        for i, waypoint in enumerate(waypoints):
            self.get_logger().info(f"导航至路径点 {i+1}/{len(waypoints)}")
            if self.navigate_to_pose(waypoint) != TaskResult.SUCCEEDED:
                self.get_logger().warn("圆形路径导航中断")
                return
        self.get_logger().info("圆形路径导航完成！")
        self.cleanup_markers()

    def run_navigation_sequence_by_waypoints(self, waypoints=None):
        """执行连续导航任务 waypoints: [(x1,y1,theta1), (x2,y2,theta2), ...] 单位：米/弧度"""
        if waypoints is None:
            self.get_logger().error("请提供路点列表： [(x1,y1,theta1), (x2,y2,theta2), ...]")
            return False
        pose_sequence = self.waypoints_to_pose_sequence(waypoints)
        # pose_sequence = self.waypoints_to_pose_sequence_with_interpolation(waypoints)
        self.publish_waypoint_markers(pose_sequence)
        for i, pose in enumerate(pose_sequence):
            self.get_logger().info(f"开始导航至路点 {i+1}/{len(waypoints)}: ({waypoints[i][0]:.2f}, {waypoints[i][1]:.2f}, {waypoints[i][2]:.2f})")
            result = self.navigate_to_pose(pose)
            if result != TaskResult.SUCCEEDED:
                self.get_logger().error(f"路点 {i+1} 导航失败，终止任务")
                return
            # 非最后一个路点时添加短暂停顿
            if i < len(waypoints) - 1:
                sleep_time = 0.5
                self.get_logger().info("准备前往下一个路点...")
                self._wait_for_duration(sleep_time)
        self.get_logger().info("所有路点导航完成！")
        self.cleanup_markers()

    def run_navigation_sequence_with_spin(self, waypoints=None):
        """执行连续导航任务，支持位置和方向的分离导航"""
        if waypoints is None:
            self.get_logger().error("请提供路点列表： [(x1,y1,theta1), (x2,y2,theta2), ...]")
            return False
        
        # 只发布位置标记
        position_poses = [self._create_pose_stamped(x, y, None) for x, y, _ in waypoints]
        self.publish_waypoint_markers(position_poses)
        
        for i, (x, y, theta) in enumerate(waypoints):
            # 第一步：导航到目标位置（方向暂时设为当前位置方向）
            position_pose = self._create_pose_stamped(x, y, None)  # 不指定方向
            self.get_logger().info(f"导航至位置: ({x:.2f}, {y:.2f})")
            position_result = self.navigate_to_pose(position_pose)
            
            if position_result != TaskResult.SUCCEEDED:
                self.get_logger().error(f"位置导航失败，终止任务")
                return
            
            # 第二步：原地旋转到目标方向
            if theta is not None:  # 检查是否需要旋转
                self.get_logger().info(f"原地旋转至方向: {theta:.2f} 弧度")
                spin_result = self.spin_in_place(theta)
                
                if spin_result != TaskResult.SUCCEEDED:
                    self.get_logger().error(f"旋转失败，终止任务")
                    return
            
            # 非最后一个路点时添加短暂停顿
            if i < len(waypoints) - 1:
                sleep_time = 0.5
                self.get_logger().info("准备前往下一个路点...")
                self._wait_for_duration(sleep_time)
        
        self.get_logger().info("所有路点导航完成！")
        self.cleanup_markers()

    def _wait_for_duration(self, seconds):
        """非阻塞式等待"""
        start = self.get_clock().now()
        while (self.get_clock().now() - start).nanoseconds < seconds * 1e9:
            rclpy.spin_once(self, timeout_sec=0.1)

    def run_navigation_sequence_continuous_by_waypoints(self, waypoints=None):
        """使用FollowWaypoints连续规划waypoints: [(x1,y1,theta1), (x2,y2,theta2), ...] 单位：米/弧度 """
        if waypoints is None:
            self.get_logger().error("必须提供waypoints参数，格式：[(x1,y1,theta1), (x2,y2,theta2),...]")
            return False
        pose_sequence = self.waypoints_to_pose_sequence(waypoints)
        self.publish_waypoint_markers(pose_sequence)
        # 调用followWaypoints执行连续导航任务
        self.get_logger().info(f"开始执行 {len(pose_sequence)} 个路点的连续导航...")
        if not self.navigator.followWaypoints(pose_sequence):
            self.get_logger().error("无法启动FollowWaypoints Action")
            return False
        # 等待任务完成
        last_printed_waypoint = -1
        while not self.navigator.isTaskComplete():
            try:
                feedback = self.navigator.getFeedback()
                # 通用进度显示方案
                current_waypoint = getattr(feedback, 'current_waypoint', -1)
                if current_waypoint != last_printed_waypoint:
                    self.get_logger().info(
                        f"正在前往路点 {current_waypoint + 1}/{len(pose_sequence)}",
                        throttle_duration_sec=1.0
                    )
                    last_printed_waypoint = current_waypoint
                    
            except Exception as e:
                self.get_logger().warn(f"反馈处理异常: {str(e)}", throttle_duration_sec=1.0)
            
            rclpy.spin_once(self, timeout_sec=0.1)

        # 结果处理
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("所有路点导航完成！")
            self.cleanup_markers()
            return True
        else:
            self.get_logger().error(f"导航失败，状态码: {result}")
            return False

def main(args=None):
    rclpy.init(args=args)
    
    try:
        navigator = AutoNavigator(node_name='auto_navigator')
        
        # 测试案例1：原地旋转测试
        navigator.spin_in_place(np.pi/2)  # 旋转90度
        navigator.spin_in_place(np.pi)    # 旋转180度
        navigator.spin_in_place(-np.pi/2) # 旋转-90度
        navigator.spin_in_place(0)        # 旋转回0度
        
        # 测试案例2：位置导航+原地旋转组合
        # waypoints = [   
        #     (1.0, 0.0, None),     # 只移动位置，不改变方向
        #     (1.0, -1.0, -1.57),   # 先到位置，再旋转到-90度
        #     (0.0, -1.0, 3.14),    # 先到位置，再旋转到180度
        #     (0.0, 0.0, 0.0),      # 先到位置，再旋转到0度
        # ]
        # navigator.run_navigation_sequence_with_spin(waypoints)
        
        # 测试案例3：圆形路径导航
        # navigator.run_navigation_cirle(center_x=0.0, center_y=-1.0, radius=1)
        
        # 测试案例4：传统连续导航（位置+方向同时移动）
        # waypoints = [   
        #     (1.0, 0.0, 0.0),    # 前进1米
        #     (1.0, -1.0, -1.57), # 左移1米，转向-90度
        #     (0.0, -1.0, 3.14),  # 后退1米，转向180度
        #     (0.0, 0.0, 0.0),    # 右移1米，转向90度
        # ]
        # navigator.run_navigation_sequence_by_waypoints(waypoints)
        
        # 测试案例5：连续路径规划（FollowWaypoints）
        # navigator.run_navigation_sequence_continuous_by_waypoints(waypoints)

    except Exception as e:
        print(f"程序异常: {str(e)}")
    finally:
        navigator.cleanup_markers()
        rclpy.shutdown()

if __name__ == '__main__':
    main()