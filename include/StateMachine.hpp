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
        /**
         * @brief 状态机添加状态(共享指针形式)
         * 
         * @param state 单一状态
         */
        void addState(std::shared_ptr<State> state)
        {
            if (state)
            {
                _states.push_back(state);
            }
        }
        /**
         * @brief 状态机添加状态（vector形式）
         * 
         * @param states 存储状态的vector数组
         * @param _append true:在原有基础上后继添加（默认）false:覆盖 
         */
        void addState(std::vector<std::shared_ptr<State>> states, bool _append = true)
        {
            if(_append)//如果为后继
            {
                _states.insert(_states.end(),states.begin(),states.end());
                return;
            }
            _states.clear();
            _states = states;//直接替换
            return;
        }
        /**
         * @brief 初始化状态机
         * 
         * @param state 初始状态
         */
        void initState(std::shared_ptr<State> state)
        {
            _currentState = state;//初始状态
        }
        /**
         * @brief 初始化状态机
         * 
         * @param pos 指针向量中对应下标的状态
         */
        void initState(int pos = 0)
        {
            _currentState = _states[pos];
        }
        /**
         * @brief 启动状态机
         * 
         */
        void start()
        {
            if(!_currentState)
            {
                return;
            }
            _currentState->onEnter();

            auto options_tmp = _currentState->onExec();
            _is_auto = options_tmp.first;
            _default_action = options_tmp.second;
            
            if (_is_auto)
            {
                this->doAction(_default_action);//自动切换状态
            }
        }
        /**
         * @brief 进行状态切换
         * 
         * @param e 动作（具体在->Actions.hpp）
         */
        void doAction(const int& e)//在这里进行状态切换
        {
            auto state = _currentState->doAction(e);
            if(!state)
            {
                return;
            }
            
            //当状态切换时
            if (_currentState != state)
            {
                _currentState->onExit();
                _currentState = state;
                _currentState->onEnter();
            }

            //无论状态是否切换，都会进行此时状态下的状态中函数
            auto options_tmp = _currentState->onExec();
            _is_auto = options_tmp.first;
            _default_action = options_tmp.second;

            //如果自动切换
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
