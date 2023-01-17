#pragma once
//状态转换类
#include "./State.hpp"
#include <memory>

namespace GFSM
{
    class Transition
    {
    private:
        int _type;//触发标志位
        std::shared_ptr<State> _state;//对应次态
    public:
        Transition():_state(nullptr){}
        Transition(const int& type, std::shared_ptr<State> state):_type(type),_state(state){}
        ~Transition() = default;
    public:
        void setType(int type)
        {
            _type = type;
        }

        int getType() const
        {
            return _type;
        }

        std::shared_ptr<State> getState() const
        {
            return _state;
        }

        void setState(std::shared_ptr<State> state)
        {
            _state = state;
        }
    };
    
} // namespace GFSM
