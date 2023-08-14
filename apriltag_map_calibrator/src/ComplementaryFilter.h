#pragma once 
#include <array>
#define POSE_DIM (7)

typedef std::array<double, POSE_DIM> POSE_ARRAY;

namespace airlab{
    class ComplementaryFilter
    {
    public:
        ComplementaryFilter(double alpha);
        void update(const std::array<double, POSE_DIM>& pose);
        POSE_ARRAY getState() const; 
        


    private:
        double _alpha;
        bool _init;
        uint32_t _count; 
        POSE_ARRAY _state;    
    };

    ComplementaryFilter::ComplementaryFilter(double alpha):
    _alpha(alpha), _init(false), _count(0)
    {

    }

    void ComplementaryFilter::update(const POSE_ARRAY& pose)
    {
        if(!_init)
        {
            std::copy(pose.begin(), pose.end(), _state.begin()); 
            _init = true; 
            return; 
        }

        for(std::size_t i = 0; i < pose.size(); ++i)
        {
            _state[i] = _alpha * _state[i] + (1 - _alpha) * pose[i]; 
        }

        ++_count; 
    }

    POSE_ARRAY ComplementaryFilter::getState() const
    {
        return _state; 
    } 
}