#pragma once
/*
    本文件为通用状态机头文件，意在于便于对哨兵的状态机的开发与调试
    主要在于便于添加状态，添加状态函数等
    在开发最后阶段可能会进行更进一步的封装，剔除init过程
*/
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <functional>
namespace GFSM
{
    class Action;  //动作信号类
    class State;  //状态类
    //class Transition;  //状态转换类 //由标准库中map代替 
    class StateManager;  //状态管理类

    //以下为类的实现
    class StateManager//状态机（管理状态运行与转换）
    {
    public:
        StateManager():_currentState(nullptr){}
        ~StateManager()
        {
            for(auto it = _states.begin();it != _states.end();)
            {
                delete *it;
                *it = nullptr;
                it++;
            }
            _states.clear();
        }

    public:
        void addState(State* state)// 添加状态
        {
            if(state)
            {
                _states.push_back(state);
            }
        }

        void setInitState(State* state)// 设置初始状态
        {
            _currentState = state;
        }

        bool start()// 开启状态机
        {
            if(!_currentState)
            {
                std::cout<<"未设置初始状态！！！"<<std::endl;
                return false;
            }
            _currentState->onEnter();
            _currentState->onExec();
            return true;
        }

        void doAction(const Action& action)// 执行接收信号后的状态处理
        {
            auto state = _currentState->doAction(action);
            if(!state)
            {
                std::cout<<"发生错误"<<std::endl;
                return;
            }
            if(_currentState != state)
            {
                _currentState->onQuit();
                _currentState = state;
                _currentState->onEnter();
            }
            _currentState->onExec();
            return;
        }
    private:
        std::vector<State*> _states;
        State* _currentState;

    };

    
    class State//状态类
    {
    private:
        std::string name; //仅作为调试使用
        std::map<int,State*> Transition; //状态转换表

    //调用函数
    private:
        std::function<void()> _enter; //进入状态时调用的函数
        std::function<void()> _exec;  //处于状态时调用
        std::function<void()> _quit;  //退出状态时调用
        

    public:
        State() = default;
        ~State()//防止内存泄漏
        {
            for (auto i = Transition.begin();i != Transition.end();)
            {
                delete i->second;
                i->second = nullptr;
                i++;
            }
            Transition.clear();
        }

        void setStateName(std::string name)//非必须
        {
            this->name = name;
        }

        std::string getStateName()
        {
            return name;
        }

        void addTransition(const int& type, State* dest)
        {
            if (dest)
            {
                Transition.insert(std::pair<int,State*>(type,dest));
            }
        }

        State* doAction(const Action& action)//返回次态
        {
            auto state_it = Transition.find(action.type());
            if(state_it != Transition.end())//若查找成功
            {
                return state_it->second;
            }
            else
            {
                std::cout<<"转换表中不存在此信号对应的状态"<<std::endl;
                return nullptr;
            }
        }
    //设置回调函数
    public:
        void setEnter(std::function<void()>& enterCall)  //设置进入状态的执行回调。
        {
            _enter = enterCall;
        }

        void setExec(std::function<void()>& execCall) //状态中
        {
            _exec = execCall;
        }

        void setQuit(std::function<void()>& quitCall) //退出状态
        {
            _quit = quitCall;
        }

        void onEnter()
        {
            if(_enter)
            {
                _enter();
            }
        }
        void onExec()
        {
            if(_exec)
            {
                _exec();
            }
        }
        void onQuit()
        {
            if(_quit)
            {
                _quit();
            }
        }
    };
    
    class Action//信号类
    {
    private:
        int _type;
    public:
        Action() = default;
        Action(const int& type)
        {
            _type = type;
        }
        ~Action() = default;
    public:
        int type() const
        {
            return _type;
        }

        void setType(const int& type)
        {
            _type = type;
        }
    };
    

    
    
} // namespace FSM
