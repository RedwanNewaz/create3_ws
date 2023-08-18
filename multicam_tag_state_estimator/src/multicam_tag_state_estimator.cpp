#include "multicam_tag_state_estimator.h"



namespace airlab
{
  multicam_tag_state_estimator::multicam_tag_state_estimator(const rclcpp::NodeOptions & options)
  : Node("multicam_tag_state_estimator", options), _image(cv::Mat())
  {
    
    this->declare_parameter("param_file", "airlab_cams.yaml");
    auto param_file =  this->get_parameter("param_file").get_parameter_value().get<std::string>();
    ApriltagDetector::init(param_file);
    

    int index = 0; 
    for(auto topic : _node_conf["camera_topics"].as<vector<string>>())
    {
      _indexMap[topic] = index; 
      RCLCPP_INFO(get_logger(), "Compressed Image Converter subscribe topic %s", topic.c_str() );

        auto subscription = this->create_subscription<sensor_msgs::msg::CompressedImage>(
          topic, 10, [this, topic](const sensor_msgs::msg::CompressedImage::SharedPtr msg){
              imageCallback(msg, _indexMap[topic]);            
            });
        subscriptions_.push_back(subscription);

        // prepare image publisher
        std::string img_topic = _frames[index] + "/detections";
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imgPub =
        this->create_publisher<sensor_msgs::msg::Image>(img_topic.c_str(), 10);
        image_pubs_.push_back(imgPub);
        
        // increment index 
        ++index;
    }

    // viz data 
    create3_state_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/apriltag/viz", 10);
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/apriltag/pose_detections", 10);
    timer_ = this->create_wall_timer(1s, [this] { publish_traj(); });
                    
  }


  void multicam_tag_state_estimator::imageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg, int index)
  {

      // RCLCPP_INFO(get_logger(), "Compressed Image  topic %d", index );
      try
      {
          cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;
          // Process the OpenCV image as needed
          processImage(image, index);
      
      }
      catch (cv_bridge::Exception &e)
      {
          RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      }
  }

  void multicam_tag_state_estimator::processImage(const cv::Mat &image, int camIndex)
  {
    std::lock_guard<std::mutex> lock(_mutex);
    std::vector<Pose> results; 

    // detect will overlay rectangles on images
    cv::Mat frame = image.clone();
    detect(frame, camIndex, results);

    // publish poses 
    geometry_msgs::msg::PoseArray poses; 
    poses.header.frame_id = "";
    poses.header.stamp = this->get_clock()->now();

    int N = results.size();

    for(auto &pose: results)
    {
      // RCLCPP_INFO_STREAM(get_logger(), pose.first.transpose());
      auto globalCoord = getMapCoord(pose, camIndex); 
      //printTransformation(globalCoord);
      state_callback(globalCoord, DEFAULT, pose.id);

      geometry_msgs::msg::Pose poseMsg; 
      tf2::toMsg(globalCoord, poseMsg);
      poses.poses.push_back(poseMsg);

      if(--N > 0)
        poses.header.frame_id = to_string(pose.id) + ",";
      else 
        poses.header.frame_id = to_string(pose.id);
      
    }

    // only publish if a tag is detected
    if(poses.poses.size() > 0)
      pose_pub_->publish(poses);
 
    // Convert OpenCV image to ROS2 sensor_msgs::Image
    sensor_msgs::msg::Image::SharedPtr rosImage = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
    image_pubs_[camIndex]->publish(*rosImage);
  }

  void multicam_tag_state_estimator::state_callback(const tf2::Transform& tf, const COLOR& color, int id)
  {
      // convert Transform to viz marker
      visualization_msgs::msg::Marker marker;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.header.stamp  = get_clock()->now();
      marker.header.frame_id  = "map";
      marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
      marker.mesh_resource = "package://irobot_create_description/meshes/body_visual.dae";
      marker.id = id;
      marker.ns = get_namespace();
      marker.scale.x = marker.scale.y = marker.scale.z = 1.0; // arrow scale 0.2 roomba scale 1.0
      switch (color) {
          case RED: marker.color.r = 1; break;
          case GREEN: marker.color.g = 1; break;
          default:
              marker.color.r = marker.color.g  = marker.color.b = 0.66;

      }
      marker.color.a = 0.85;
      tf2::toMsg(tf, marker.pose);
      create3_state_pub_->publish(marker);
      pubData_[id].push_back(marker.pose.position);
  }
  
  void multicam_tag_state_estimator::publish_traj()
  {
      // it is problematic if we continuously publish long trajectory use timer function to periodically publish trajectory
      for(auto item: pubData_)
      {
        int _id = item.first; 
        visualization_msgs::msg::Marker trajMarker;
        std::copy(item.second.begin(), item.second.end(), std::back_inserter(trajMarker.points));
        trajMarker.id = _id + 202;
        trajMarker.action = visualization_msgs::msg::Marker::ADD;
        trajMarker.header.stamp = get_clock()->now();
        trajMarker.header.frame_id = "map";

        trajMarker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        trajMarker.ns = get_namespace();
        trajMarker.color.g = 1;
        trajMarker.color.b = 1;
        trajMarker.color.r = _id / 32;
        trajMarker.color.a = 0.7;
        trajMarker.scale.x = trajMarker.scale.y = trajMarker.scale.z = 0.08;
        if(trajMarker.points.size() > 0)
          create3_state_pub_->publish(trajMarker);
      }

  
  }

}


RCLCPP_COMPONENTS_REGISTER_NODE(airlab::multicam_tag_state_estimator)

