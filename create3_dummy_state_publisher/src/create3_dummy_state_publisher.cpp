#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Transform.h>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <rclcpp_components/register_node_macro.hpp>
#include <chrono>

namespace airlab{
  class create3_dummy_state_publisher: public rclcpp::Node
  {
  public:
    create3_dummy_state_publisher(const rclcpp::NodeOptions& options): 
    rclcpp::Node("create3_dummy_state_publisher",options), dt_(0.03)
    {
      this->declare_parameter("start_loc", std::vector<double>{-1, -2});
      this->declare_parameter("tag_id", 32);

      tag_id_ = this->get_parameter("tag_id").get_parameter_value().get<int>();
      auto pos = this->get_parameter("start_loc").get_parameter_value().get<std::vector<double>>();
      auto origin = tf2::Vector3(pos[0], pos[1], 0);
      tf2::Quaternion q(0, 0, 0.0, 1.0);  
      robotState_.setOrigin(origin);
      robotState_.setRotation(q);

      create3_state_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/apriltag/viz/filtered", 10);
      robot_state_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("apriltag/state/filtered", 10);

      cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, [this](geometry_msgs::msg::Twist::SharedPtr msg)
      {
        double v = msg->linear.x; 
        double w = msg->angular.z; 

        tf2::Quaternion q = robotState_.getRotation();
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw; 
        m.getRPY(roll, pitch, yaw); 
        yaw = yaw + dt_ * w;
        yaw = fmod(yaw, 2 * M_PI);
        q.setRPY(0, 0, yaw); 
        robotState_.setRotation(q);
        
        double dx = v * cos(yaw) * dt_; 
        double dy = v * sin(yaw) * dt_; 
        tf2::Vector3 dorigin(dx, dy, 0); 
        tf2::Vector3 origin = robotState_.getOrigin() + dorigin; 
        robotState_.setOrigin(origin);

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = get_clock()->now();
        odom.header.frame_id = "map";
        tf2::toMsg(robotState_, odom.pose.pose);
        robot_state_pub_->publish(odom);


      });

      state_loop_ = this->create_wall_timer(std::chrono::milliseconds(250), [this] { 
         auto origin = robotState_.getOrigin(); 
         auto q = robotState_.getRotation(); 

         tf2::Matrix3x3 m(q);
         double roll, pitch, yaw; 
         m.getRPY(roll, pitch, yaw); 
         q.setRPY(0, 0, yaw + M_PI_2); 
         
         tf2::Transform newState; 
         newState.setOrigin(origin); 
         newState.setRotation(q);
         
         state_callback(newState);              
      });
      RCLCPP_INFO(get_logger(), "create3_dummy_state_publisher started");
    }
  private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr create3_state_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr robot_state_pub_;  
    rclcpp::TimerBase::SharedPtr state_loop_;
    tf2::Transform robotState_; 
    double dt_;
    int tag_id_;
  protected:
    void state_callback(const tf2::Transform& tf)
    {
        // convert Transform to viz marker
        visualization_msgs::msg::Marker marker;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.header.stamp  = get_clock()->now();
        marker.header.frame_id  = "map";
        marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
        marker.mesh_resource = "package://irobot_create_description/meshes/body_visual.dae";
        marker.id = tag_id_;
        marker.ns = get_namespace();
        marker.scale.x = marker.scale.y = marker.scale.z = 1.0; // arrow scale 0.2 roomba scale 1.0
        marker.color.r = marker.color.g  = marker.color.b = 0.66;
        marker.color.a = 0.85;
        tf2::toMsg(tf, marker.pose);
        create3_state_pub_->publish(marker);
    } 
  };
}

RCLCPP_COMPONENTS_REGISTER_NODE(airlab::create3_dummy_state_publisher)