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
        auto M_T_C1tag = _cam1->getPose();
        auto M_T_C2tag = _cam2->getPose();
        // by definition both should be pointing the map location 
        auto C1_pos_err = _mapTF.getOrigin() - M_T_C1tag.getOrigin(); 
        auto C1_ori_err = _mapTF.getRotation() - M_T_C1tag.getRotation(); 
    }

}