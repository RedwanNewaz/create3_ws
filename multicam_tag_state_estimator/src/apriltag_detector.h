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
  
  protected:
    void detect(const cv::Mat& image, int camIndex, std::vector<Pose>& results) override;

    Pose getPose(const matd_t& H, int camIndex);
  
  private:
    apriltag_detector_t* const _td;

    void decode_results(zarray_t* detections, int camIndex, std::vector<Pose>& results);

    cv::Mat annotate_image(const cv::Mat& image, zarray_t* detections);

    Pose getPose(const matd_t& H,  const Mat3& Pinv,  const double size) const; 
  };


  ApriltagDetector::ApriltagDetector():_td(apriltag_detector_create())
  {
    apriltag_family_t *tf = tag36h11_create();
    apriltag_detector_add_family(_td, tf);
    _td->quad_decimate = 2.0;
    _td->quad_sigma = 0.1;
    _td->nthreads = 16;
    _td->debug = false;
    _td->refine_edges = true;
  }


  void ApriltagDetector::detect(const cv::Mat& image, int camIndex, std::vector<Pose>& results)
  {
    cv::Mat img_uint8;
    cv::cvtColor(image, img_uint8, cv::COLOR_BGR2GRAY);
    
    image_u8_t im{img_uint8.cols, img_uint8.rows, img_uint8.cols, img_uint8.data};
    zarray_t* detections = apriltag_detector_detect(_td, &im);
    decode_results(detections, camIndex, results);
    
    auto frame = annotate_image(image, detections);


    cv::imshow("frame" + std::to_string(camIndex), frame); 
    cv::waitKey(1);

  }

 
  cv::Mat ApriltagDetector::annotate_image(const cv::Mat& image, zarray_t* detections)
  {
    auto frame = image.clone();
    // Draw detection outlines
    for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);
        cv::line(frame, cv::Point(det->p[0][0], det->p[0][1]),
                  cv::Point(det->p[1][0], det->p[1][1]),
                  cv::Scalar(0, 0xff, 0), 2);
        cv::line(frame, cv::Point(det->p[0][0], det->p[0][1]),
                  cv::Point(det->p[3][0], det->p[3][1]),
                  cv::Scalar(0, 0, 0xff), 2);
        cv::line(frame, cv::Point(det->p[1][0], det->p[1][1]),
                  cv::Point(det->p[2][0], det->p[2][1]),
                  cv::Scalar(0xff, 0, 0), 2);
        cv::line(frame, cv::Point(det->p[2][0], det->p[2][1]),
                  cv::Point(det->p[3][0], det->p[3][1]),
                  cv::Scalar(0xff, 0, 0), 2);

        stringstream ss;
        ss << det->id;
        String text = ss.str();
        int fontface = FONT_HERSHEY_SCRIPT_SIMPLEX;
        double fontscale = 1.0;
        int baseline;
        cv::Size textsize = cv::getTextSize(text, fontface, fontscale, 2,
                                        &baseline);
        cv::putText(frame, text, cv::Point(det->c[0]-textsize.width/2,
                                    det->c[1]+textsize.height/2),
                fontface, fontscale, cv::Scalar(0xff, 0x99, 0), 2);
    }
    return frame; 
  }

  void ApriltagDetector::decode_results(zarray_t* detections, int camIndex, std::vector<Pose>& results)
  {
    
    for(int i = 0; i < zarray_size(detections); i++) {
      
      apriltag_detection_t* det;
      zarray_get(detections, i, &det);
      // printf("detection %3d: id (%2dx%2d)-%-4d, hamming %d, margin %8.3f\n",
      //                i, det->family->nbits, det->family->h, det->id,
      //                det->hamming, det->decision_margin);

      auto pose = getPose(*(det->H), camIndex);
      results.emplace_back(pose);
    }

  }

  ApriltagDetector::Pose ApriltagDetector::getPose(const matd_t& H, int camIndex)
  {
    auto tag = _configs.at(camIndex);
    return getPose(H, tag.PINV, tag.TAG_SIZE);        
  }  

   ApriltagDetector::Pose ApriltagDetector::getPose(const matd_t& H,
              const Mat3& Pinv,
              const double size) const 
      {
          // compute extrinsic camera parameter
          // https://dsp.stackexchange.com/a/2737/31703
          // H = K * T  =>  T = K^(-1) * H
          const Mat3 T = Pinv * Eigen::Map<const Mat3>(H.data);
          Mat3 R;
          R.col(0) = T.col(0).normalized();
          R.col(1) = T.col(1).normalized();
          R.col(2) = R.col(0).cross(R.col(1));

          // rotate by half rotation about x-axis to have z-axis
          // point upwards orthogonal to the tag plane
          R.col(1) *= -1;
          R.col(2) *= -1;

          // the corner coordinates of the tag in the canonical frame are (+/-1, +/-1)
          // hence the scale is half of the edge size
          const Eigen::Vector3d tt = T.rightCols<1>() / ((T.col(0).norm() + T.col(0).norm()) / 2.0) * (size / 2.0);

          const Eigen::Quaterniond q(R);

          return std::make_pair(tt, q);
      }


}