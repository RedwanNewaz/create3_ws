#include <cstdio>
#include <iostream>
#include <iomanip>
#include <atomic>

#include "apriltag_detector.h"

#include<opencv2/opencv.hpp>


#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <filesystem>
#include "sensor_msgs/msg/compressed_image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


using namespace std;
using namespace cv;

namespace airlab{
  

  

class CompressedImageConverter : public rclcpp::Node, public ApriltagDetector
{
  using CAM_SUBS = rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr;
public:
    
    CompressedImageConverter()
        : Node("compressed_image_converter"), _image(cv::Mat())
    {
      int index = 0; 
      for(auto topic : _node_conf["camera_topics"].as<vector<string>>())
      {
        std::string windowName = "frame" + to_string(index);
        namedWindow(windowName, WINDOW_NORMAL);
        resizeWindow(windowName, 800, 600); // Set the desired width and height
        
        _indexMap[topic] = index++; 
        RCLCPP_INFO(get_logger(), "Compressed Image Converter subscribe topic %s", topic.c_str() );

          auto subscription = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            topic, 10, [this, topic](const sensor_msgs::msg::CompressedImage::SharedPtr msg){
                imageCallback(msg, _indexMap[topic]);            
              });
          subscriptions_.push_back(subscription);
      }

      // viz data 
      create3_state_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/apriltag/viz", 10);
      timer_ = this->create_wall_timer(1s, [this] { publish_traj(); });
            
        
    }

private:
    enum COLOR{
            RED,
            GREEN,
            DEFAULT
        };
    std::unordered_map<string, int> _indexMap; 
    void imageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg, int index)
    {

        // RCLCPP_INFO(get_logger(), "Compressed Image  topic %d", index );
        try
        {
            cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;
            // Process the OpenCV image as needed
            processImage(image.clone(), index);
         
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    void processImage(const cv::Mat &image, int camIndex)
    {
      std::lock_guard<std::mutex> lock(_mutex);
      std::vector<Pose> results; 


      detect(image, camIndex, results);
      for(auto &pose: results)
      {
        // RCLCPP_INFO_STREAM(get_logger(), pose.first.transpose());
        auto globalCoord = getMapCoord(pose, camIndex); 
        printTransformation(globalCoord);
        state_callback(globalCoord, DEFAULT);
      }
    }

    void state_callback(const tf2::Transform& tf, const COLOR& color)
    {
        // convert Transform to viz marker
        visualization_msgs::msg::Marker marker;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.header.stamp  = get_clock()->now();
        marker.header.frame_id  = "map";
        marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
        marker.mesh_resource = "package://irobot_create_description/meshes/body_visual.dae";
        marker.id = static_cast<int>(color);
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
        pubData_[color].push_back(marker.pose.position);
    }
    
      void publish_traj()
      {
          // it is problematic if we continuously publish long trajectory use timer function to periodically publish trajectory
          auto color = DEFAULT;
          if(pubData_[color].empty())
              return;

          visualization_msgs::msg::Marker trajMarker;
          std::copy(pubData_[color].begin(), pubData_[color].end(), std::back_inserter(trajMarker.points));
          trajMarker.id = 202;
          trajMarker.action = visualization_msgs::msg::Marker::ADD;
          trajMarker.header.stamp = get_clock()->now();
          trajMarker.header.frame_id = "map";

          trajMarker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
          trajMarker.ns = get_namespace();
          trajMarker.color.g = 1;
          trajMarker.color.b = 1;
          trajMarker.color.a = 0.7;
          trajMarker.scale.x = trajMarker.scale.y = trajMarker.scale.z = 0.08;
          create3_state_pub_->publish(trajMarker);
      }

    std::vector<CAM_SUBS> subscriptions_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr create3_state_pub_;
    std::unordered_map<COLOR, std::vector<geometry_msgs::msg::Point>> pubData_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::Mat _image;
    mutable std::mutex _mutex; 
    
};
}




int main(int argc, char ** argv)
{


  printf("hello world multicam_tag_state_estimator package\n");

   
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<airlab::CompressedImageConverter>());
  rclcpp::shutdown();

  

  return 0;
}
