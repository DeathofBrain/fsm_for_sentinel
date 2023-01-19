#pragma once

#include "State.hpp"

namespace GFSM
{
    class StateMachine
    {
    private:
        std::vector<std::shared_ptr<State>> _states;
        std::shared_ptr<State> _currentState = nullptr;
        bool _is_auto = false;//自动切换开关，理论上应当由*状态*自行控制
        int _default_action = -1;//默认为-1，即没有默认信号，理论上应当由*状态*自行控制
    public:
        StateMachine() = default;
        ~StateMachine() = default;
    public:
        void addState(std::shared_ptr<State> state)//添加状态
        {
            if (state)
            {
                _states.push_back(state);
            }
            
        }

        void initState(std::shared_ptr<State> state)//初始化状态机
        {
            _currentState = state;//初始状态
            // auto pair = state->getOptions();//以下：设定状态机的自动切换开关，默认次态
            // _is_auto = pair.first;
            // _default_action = pair.second;
        }

        void start()
        {
            if(!_currentState)
            {
                return;
            }
            _currentState->onEnter();

            auto options_tmp = _currentState->onExec();//TODO:让_exec返回int与bool（即pair），让状态自行决定是否自动运行
            _is_auto = options_tmp.first;
            _default_action = options_tmp.second;
            
            if (_is_auto)
            {
                this->doAction(_default_action);//自动切换状态
            }
        }

        void doAction(const int& e)//在这里进行状态切换
        {
            auto state = _currentState->doAction(e);
            if(!state)
            {
                return;
            }
            if (_currentState != state)//当状态切换时
            {
                _currentState->onExit();

                _currentState = state;
                // auto pair = state->getOptions();
                // _is_auto = pair.first;
                // _default_action = pair.second;

                _currentState->onEnter();
            }

            auto options_tmp = _currentState->onExec();//无论状态是否切换，都会进行此时状态下的状态中函数
            _is_auto = options_tmp.first;
            _default_action = options_tmp.second;
            
            if (_is_auto)
            {
                this->doAction(_default_action);
            }
        }
    public:
        static void changeMode(StateMachine& sm,const bool& is_auto)//留了个开关api,后门
        {
            sm._is_auto = is_auto;
        }

        static void changeDefaultAction(StateMachine& sm,const int& action)//后门
        {
            sm._default_action = action;
        }
    };
    

    
} // namespace GFSM
