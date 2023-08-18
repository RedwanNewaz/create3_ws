#pragma once 

#include <iostream>
#include "detector_base.h"
#include <apriltag.h>
#include <tag36h11.h>


namespace airlab{
  class ApriltagDetector : public DetectorBase
  {
  public:
    ApriltagDetector();
    void init(const std::string& param_file);
  
  protected:
    void detect(cv::Mat& image, int camIndex, std::vector<Pose>& results) override;

    Pose getPose(const matd_t& H, int camIndex);
  
  private:
    apriltag_detector_t* const _td;

    void decode_results(zarray_t* detections, int camIndex, std::vector<Pose>& results);

    void annotate_image(cv::Mat& image, zarray_t* detections);

    Pose getPose(const matd_t& H,  const Mat3& Pinv,  const double size) const; 
  };
}