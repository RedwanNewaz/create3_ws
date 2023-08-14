#pragma once 
#include "MapTransformer.h"

namespace airlab{
    class GradientDescent{
        using MapTransPtr = std::shared_ptr<MapTransformer>; 
    public:
        GradientDescent(const tf2::Transform& mapTF, MapTransPtr cam1, MapTransPtr cam2, double learning_rate);
        void computeGradient(); 
    private:
        MapTransPtr _cam1, _cam2; 
        tf2::Transform _mapTF; 
        double _lr; 

    };

    GradientDescent::GradientDescent(const tf2::Transform& mapTF, MapTransPtr cam1, MapTransPtr cam2, double learning_rate)
    : _mapTF(mapTF), _cam1(cam1), _cam2(cam2), _lr(learning_rate)
    {

    }

    void GradientDescent::computeGradient()
    {
        auto cam1_T_tag = _cam1->getPose();
        auto cam2_T_tag = _cam2->getPose();
        

        // estimate tag_T_cam1 with respect to cam2 
        auto tag_T_cam2 = cam2_T_tag.inverse();
        auto cam1_T_cam2 = cam1_T_tag * tag_T_cam2;
        
        // estimate tag_T_cam2 with respect to cam1 
        auto tag_T_cam1 = cam1_T_tag.inverse();
        auto cam2_T_cam1 = cam2_T_tag * tag_T_cam1;
        
        
        auto tag_T_cam1_ =  tag_T_cam2 * cam2_T_cam1;
        auto tag_T_cam2_ =  tag_T_cam1 * cam1_T_cam2;
        

    }

}