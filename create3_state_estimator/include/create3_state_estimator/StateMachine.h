#pragma once 

namespace model{
    class StateMachine
    {
        int state_;
    public:
        StateMachine():state_(0)
        {

        }
        int update(bool logitec, bool nexigo)
        {

            if(state_ == 0 && nexigo && !logitec)
            {
                state_ = 2;
            }
            if(state_ == 0 && !nexigo && logitec)
            {
                state_ = 1;
            }

            if(state_ == 0 && nexigo && logitec)
            {
                state_ = 2; // priority goes to nexigo
            }

            if(state_ == 1 &&  nexigo && logitec)
            {
                state_ = 1;
            }

            if(state_ == 1 &&  !nexigo && logitec)
            {
                state_ = 1;
            }

            if(state_ == 1 && nexigo && !logitec)
            {
                state_ = 2; // nexigo
            }


            if(state_ == 2 && nexigo && !logitec)
            {
                state_ = 2;
            }
            if(state_ == 2 && !nexigo && logitec)
            {
                state_ = 1;
            }
            if(state_ == 2 && nexigo && logitec)
            {
                state_ = 2;
            }



            if (!logitec && !nexigo)
            {
                state_ = 0;
            }
            return state_;
        }

    };
}