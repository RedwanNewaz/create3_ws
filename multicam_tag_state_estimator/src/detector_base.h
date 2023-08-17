#pragma once 

#include <iostream>
#include <Eigen/Dense>
#include<opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <tf2/LinearMath/Transform.h>

using namespace std;
using namespace cv;

namespace airlab{
    inline void printTransformation(const tf2::Transform& transform)
    {
        std::cout << "Translation: (" << transform.getOrigin().x()
                << ", " << transform.getOrigin().y()
                << ", " << transform.getOrigin().z() << ")" << std::endl;
        std::cout << "Rotation: (" << transform.getRotation().x()
                << ", " << transform.getRotation().y()
                << ", " << transform.getRotation().z()
                << ", " << transform.getRotation().w() << ")" << std::endl;
    }

    class DetectorBase{

    public:
      using Mat3 = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>;
      using Pose = std::pair<Eigen::Vector3d, Eigen::Quaterniond>;
      struct TagConfig{Mat3 PINV; double TAG_SIZE;};
      DetectorBase()
      {

        std::string package_name = "multicam_tag_state_estimator";
        std::string package_path = ament_index_cpp::get_package_share_directory(package_name);
        
        auto config_path = package_path + "/config/" + "airlab_cams.yaml";
        
        cout << config_path << endl; 

        _node_conf = YAML::LoadFile(config_path);
        for (auto& cam_info: _node_conf["camera_configs"].as<std::vector<std::string>>())
        {
          // read camera info   
          auto config_info = package_path + "/config/" + cam_info;
          YAML::Node cam_conf = YAML::LoadFile(config_info);
          _camConfs.emplace_back(cam_conf);
        
          // track camera projection matrix to calculate realworld coordinate 
          auto projection = cam_conf["projection_matrix"]["data"].as<std::vector<double>>();  
          double tagSize = _node_conf["tag_size"].as<double>();
          auto tag = TagConfig{getPINV(projection), tagSize};
          _configs.emplace_back(tag); 
        }

        // get map Transformations 
        for(YAML::const_iterator it = _node_conf["transformation"].begin(); it != _node_conf["transformation"].end();++it) {

            auto tfValues = it->second.as<std::map<std::string, std::vector<double>>>();
            _mapTfs.emplace_back(convertTFfromVector(tfValues["map"]));
        }

      }

      

    protected:      
        std::vector<YAML::Node> _camConfs;
        std::vector<tf2::Transform> _mapTfs;
        std::vector<TagConfig> _configs; 
        YAML::Node _node_conf;       

    protected:
        virtual void detect(const cv::Mat& img_uint8, int camIndex, std::vector<Pose>& results) = 0;

        tf2::Transform getMapCoord(const Pose& pose, int camIndex)
        {
            return getMapCoord(convertTFfromPose(pose), _mapTfs[camIndex]);
        }



    private:        
        
        Mat3 getPINV(const std::vector<double>&data) const 
        {
            return Eigen::Map<const Eigen::Matrix<double, 3, 4, Eigen::RowMajor>>(data.data()).leftCols<3>().inverse();
        }

        tf2::Transform convertTFfromPose(const Pose& pose)
        {
            auto p = pose.first; 
            auto q = pose.second; 
            tf2::Transform transform; 

            tf2::Vector3 origin(p(0), p(1), p(2));
            tf2::Quaternion rotation(q.x(), q.y(), q.z(), q.w()); 

            transform.setOrigin(origin);
            transform.setRotation(rotation); 

            return transform; 
        }

        tf2::Transform convertTFfromVector(const std::vector<double>& pose)
        {
   
            tf2::Transform transform; 

            tf2::Vector3 origin(pose.at(0), pose.at(1), pose.at(2));
            tf2::Quaternion rotation(pose.at(3), pose.at(4), pose.at(5), pose.at(6)); 

            transform.setOrigin(origin);
            transform.setRotation(rotation); 

            return transform; 
        }

        tf2::Transform getMapCoord(const tf2::Transform& pose, const tf2::Transform& C_T_M)
        {
            // pose is C_T_R : camera to robot transformation 
            // C_T_M : camera to map transformation
            // need to find M_T_R transformation 
            tf2::Transform M_T_R = C_T_M.inverseTimes(pose);
            return M_T_R; 
        }  
     
  };
}