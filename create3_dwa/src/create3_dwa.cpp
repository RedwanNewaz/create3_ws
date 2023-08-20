#include "create3_dwa.h"

namespace airlab{
  create3_dwa::create3_dwa(const rclcpp::NodeOptions & options)
  :Node("create3_dwa", options), initialized_(false)
  {
    this->declare_parameter("tag_id", 32);
    tag_id_ = this->get_parameter("tag_id").get_parameter_value().get<int>();
      
    // "apriltag/state/filtered"
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("apriltag/state/filtered", 10, [&](nav_msgs::msg::Odometry::SharedPtr msg)
    {
      auto p = msg->pose.pose.position; 
      // RCLCPP_INFO(get_logger(), "state = (%lf, %lf)", p.x, p.y);
      tf2::fromMsg(msg->pose.pose, robot_pose_);
    });

    rviz_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("goal_pose", 10, [this](geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
      auto p = msg->pose.position; 
      RCLCPP_INFO(get_logger(), "[goal_pose] target := (%lf, %lf)", p.x, p.y);
      tf2::fromMsg(msg->pose, goal_pose_);
      initialized_ = true; 
    });

    traj_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/apriltag/viz/filtered", 10);

    obs_sub_ = this->create_subscription<visualization_msgs::msg::Marker>("/apriltag/viz/filtered", 10, [this](visualization_msgs::msg::Marker::SharedPtr msg)
    {
      std::string ns = msg->ns;
      std::string self = get_namespace(); 
      if(ns == self)
        return; 
      // RCLCPP_INFO(get_logger(), "self = %s , other = %s", self.c_str(), ns.c_str());     
      // if(!msg->points.empty())
      auto name = ns + std::to_string(msg->id);
      auto pose_var = ns + "pose";
      obstacleMap_[pose_var].clear();
      obstacleMap_[pose_var].push_back(msg->pose.position);
      obstacles_callback(name, msg->points);

    });

    cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",1);
    

    control_loop_ = this->create_wall_timer(std::chrono::milliseconds(30), [this] { 
      if(initialized_)
        execute();
    });

    rviz_loop_ = this->create_wall_timer(std::chrono::milliseconds(250), [this] { 
      if(initialized_)
      {
        if(ltraj_.size() > 0)
          publish_short_horizon_traj(ltraj_);
        state_callback(robot_pose_);
      }
    });
    // TODO get parameter 
    std::string package_name = "create3_dwa";
    std::string package_path = ament_index_cpp::get_package_share_directory(package_name);
    
    auto config_path = package_path + "/config/" + "dwa_param.yaml";
    parameters_ = std::make_shared<param_manager2>(config_path);
     
    std::function<double(const std::string&)> f = [&](const std::string& param)
    {return parameters_->get_param<double>(param);};
    config_.update_param(f);
    
  }

  void create3_dwa::execute() {

    std::lock_guard<std::mutex> lock(mu_);
    // compute state difference
    auto goal_position = goal_pose_.getOrigin();
    auto curr_position = robot_pose_.getOrigin();
    tf2::Vector3 position_diff = goal_position - curr_position;

    auto alpha = atan2(position_diff.y(), position_diff.x());

    double roll, pitch, yaw, current_angle;
    tf2::Matrix3x3 m(robot_pose_.getRotation());
    m.getRPY(roll, pitch, yaw);
    current_angle = yaw;
    // current_angle = fmod(yaw - M_PI_2, 2 * M_PI);
//    RCLCPP_INFO(get_logger(), "goal heading angle = %lf", goal_heading);

    // compute terminal condition
    double remainDist = sqrt(pow(position_diff.x(), 2) + pow(position_diff.y(), 2) );
    Point goal;
    goal[0] = goal_position.x();
    goal[1] = goal_position.y();

    // don't move if you reach to a goal region
    const double goal_radius = 0.3; // m
    if (remainDist < goal_radius)
    {
        if(initialized_)
            RCLCPP_INFO(get_logger(), "[dwa_controller] goal reached :-)");
        initialized_ = false;
        // handover the controller to the joystick
        control_[0] = control_[1] = 0;
        return;
    }
    else
    {
        // compute local trajectory using dynamic window
        State x({{curr_position.x(), curr_position.y(), current_angle, control_[0], control_[1]}});
        ltraj_ = dwa_.compute_control(x, control_, config_, goal, obstacles_);
        //publish_short_horizon_traj(ltraj_);
    }

    alpha = alpha  - current_angle  + M_PI;
    alpha = fmod(alpha, 2 * M_PI) - M_PI;

    if (alpha > M_PI_2)
    {
        control_[0] *= 0;
        control_[1] = 0.5;
    }
    else if (alpha < - M_PI_2)
    {
      control_[0] *= 0;
      control_[1] = -0.5;
    }

    // publish command 
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = control_[0];
    cmd_vel.angular.z = control_[1];
    cmd_vel_->publish(cmd_vel);


  }

  void create3_dwa::publish_short_horizon_traj(Traj &traj) {
    visualization_msgs::msg::Marker msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "map";
    msg.id = 302 + tag_id_;
    msg.action = visualization_msgs::msg::Marker::ADD;
    for(auto & p: traj)
    {
        geometry_msgs::msg::Point pose;
        pose.x = p[0];
        pose.y = p[1];
        pose.z = 0;
        msg.points.emplace_back(pose);      
    }

    msg.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    msg.ns = get_namespace();
    msg.color.g = 1;
    msg.color.a = 0.7;
    msg.scale.x = msg.scale.y = msg.scale.z = 0.04;

    geometry_msgs::msg::Pose goal;
    tf2::toMsg(goal_pose_, goal);
    msg.points.emplace_back(goal.position);

    traj_pub_->publish(msg);
    
  }

  void create3_dwa::obstacles_callback(const std::string& ns, const std::vector<geometry_msgs::msg::Point>& points)
  {
    std::lock_guard<std::mutex> lock(mu_);
    if(obstacleMap_.find(ns) != obstacleMap_.end())
      obstacleMap_[ns].clear();
    
    std::copy(points.begin(), points.end()-1, std::back_inserter(obstacleMap_[ns]));  

    if(!obstacles_.empty())
      obstacles_.clear();
    
    for (const auto& obst: obstacleMap_)
    {
      // // the last point is the goal location 
      for(int i = 0; i < static_cast<int>(obst.second.size()); ++i)
      {
        //convert point to obstacle 
        auto point = obst.second[i]; 
        std::array<double, 2> obs{point.x, point.y};
        obstacles_.emplace_back(obs);
      }
    }

  }

  void create3_dwa::state_callback(const tf2::Transform& tf)
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

      auto origin = tf.getOrigin(); 
      auto q = tf.getRotation(); 

      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw; 
      m.getRPY(roll, pitch, yaw); 
      q.setRPY(0, 0, yaw + M_PI_2); 
      
      tf2::Transform newState; 
      newState.setOrigin(origin); 
      newState.setRotation(q);

      tf2::toMsg(newState, marker.pose);
      traj_pub_->publish(marker);
  } 

}

RCLCPP_COMPONENTS_REGISTER_NODE(airlab::create3_dwa)

