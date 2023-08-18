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
      struct Pose {Eigen::Vector3d translation; Eigen::Quaterniond rotation; int id;};
      struct TagConfig{Mat3 PINV; double TAG_SIZE;};
      DetectorBase()
      {
        // init("airlab_cams.yaml");
        
      }

      void init(const std::string& param_file)
      {
        std::string package_name = "multicam_tag_state_estimator";
        std::string package_path = ament_index_cpp::get_package_share_directory(package_name);
        
        auto config_path = package_path + "/config/" + param_file;
        
        // cout << config_path << endl; 

        _node_conf = YAML::LoadFile(config_path);
        for (auto& cam_info: _node_conf["camera_configs"].as<std::vector<std::string>>())
        {
          // read camera info   
          auto config_info = package_path + "/config/" + cam_info;
          YAML::Node cam_conf = YAML::LoadFile(config_info);
          _camConfs.emplace_back(cam_conf);

          // get camera name 
          _frames.emplace_back(cam_conf["camera_name"].as<std::string>());
        
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

        readApriltagParams();

        _calibration = _node_conf["calibration"].as<bool>();
      }

      

    protected:      
        std::vector<YAML::Node> _camConfs;
        std::vector<tf2::Transform> _mapTfs;
        std::vector<TagConfig> _configs;
        std::vector<std::string> _frames; 
        YAML::Node _node_conf; 

    protected:
        double _quad_decimate;
        double _quad_sigma; 
        int _nthreads; 
        bool _refine_edges;
        bool _calibration;     

    protected:
        virtual void detect(cv::Mat& img_uint8, int camIndex, std::vector<Pose>& results) = 0;

        tf2::Transform getMapCoord(const Pose& pose, int camIndex)
        {
            return getMapCoord(convertTFfromPose(pose), _mapTfs[camIndex]);
        }
        
        tf2::Transform convertTFfromPose(const Pose& pose)
        {
            auto p = pose.translation; 
            auto q = pose.rotation; 
            tf2::Transform transform; 

            tf2::Vector3 origin(p(0), p(1), p(2));
            tf2::Quaternion rotation(q.x(), q.y(), q.z(), q.w()); 

            transform.setOrigin(origin);
            transform.setRotation(rotation); 

            return transform; 
        }


    private:    

        void readApriltagParams()
        {
            _quad_decimate = _node_conf["quad_decimate"].as<double>();
            _quad_sigma = _node_conf["quad_sigma"].as<double>();
            _nthreads = _node_conf["nthreads"].as<int>();
            _refine_edges = _node_conf["refine_edges"].as<bool>();

        }    
        
        Mat3 getPINV(const std::vector<double>&data) const 
        {
            return Eigen::Map<const Eigen::Matrix<double, 3, 4, Eigen::RowMajor>>(data.data()).leftCols<3>().inverse();
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
            
            // fix yaw angle 
            auto q = M_T_R.getRotation(); 
            tf2::Matrix3x3 m(q); 
            double roll, pitch, yaw; 
            m.getRPY(roll, pitch, yaw); 
            q.setRPY(0, 0, yaw + M_PI);
            M_T_R.setRotation(q);

            return M_T_R; 
        }  
     
  };
}