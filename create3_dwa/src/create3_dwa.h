#pragma once 

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <mutex> 
#include <chrono>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "dwa_planner.h"
#include "param_manager2.h"

namespace airlab{

  class create3_dwa: public rclcpp::Node{
  public:
    create3_dwa(const rclcpp::NodeOptions& options);

  protected:
    void execute();
    void publish_short_horizon_traj(Traj &traj);
    void obstacles_callback(const std::string& ns, const std::vector<geometry_msgs::msg::Point>& obstacles);
    void state_callback(const tf2::Transform& tf);
  private:
    bool initialized_; 
    DynamicWindow::Config config_;
    DynamicWindow::planner dwa_;
    Control control_;
    Obstacle obstacles_;
    ParamPtr2 parameters_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr traj_pub_;
    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr obs_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr rviz_sub_;
    rclcpp::TimerBase::SharedPtr control_loop_;
    
    mutable std::mutex mu_; 
    int tag_id_;

    tf2::Transform robot_pose_;
    tf2::Transform goal_pose_;
    std::unordered_map<std::string, std::vector<geometry_msgs::msg::Point>> obstacleMap_;
  };
}
