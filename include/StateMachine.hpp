#pragma once

#include "./State.hpp"
#include "./Actions.hpp"
namespace GFSM
{
    /**
     * @brief 状态机类
     * 
     */
    class StateMachine
    {
    private:
        /**
         * @brief 存储状态的动态数组
         * 
         */
        std::vector<std::shared_ptr<State>> _states;
        /**
         * @brief 状态机当前状态
         * 
         */
        std::shared_ptr<State> _currentState = nullptr;
        /**
         * @brief 自动切换开关，理论上应当由*状态*自行控制
         * 
         */
        bool _is_auto = false;
        /**
         * @brief 自动切换时自动传递的信号，由状态自行控制，默认为无动作(-1)
         * 
         */
        int _default_action = Actions::NO_DEFAULT_ACTION;
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
