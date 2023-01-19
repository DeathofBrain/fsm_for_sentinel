#pragma once
//状态类
#include "./Transition.hpp"


#include <string>
#include <vector>
#include <functional>
#include <memory>


namespace GFSM
{
    class State
    {
    private:
        std::string _name;//用于后期确认状态是否符合预期
        // bool _is_auto = false;//默认为取消，控制**该状态下**是否需要自动切换状态
        // int _default_action = -1;//默认为-1，即没有默认信号，由*状态*自行控制
    private:
        std::vector<std::shared_ptr<Transition>> _trans;//此状态对应的信号——次态集合
        std::function<void()> _enter;
        std::function<std::pair<bool,int>()> _exec;
        std::function<void()> _exit;
    public:
        State() = default;
        State(const std::string& name):_name(name){}
        ~State() = default;
        std::string getName() const
        {
            return _name;
        }
    public:
        void addTransition(const int& type, std::shared_ptr<State> dest)
        {
            std::shared_ptr<Transition> transition = std::make_shared<Transition>(type,dest);
            _trans.push_back(transition);
        }

        std::shared_ptr<State> doAction(const int& e)
        {
            std::shared_ptr<Transition> trans;

            for (auto it : _trans)
            {
                if ((it)->getType() == e)
                {
                    trans = it;
                    break;
                }
            }
            std::shared_ptr<State> state;
            if (trans)
            {
                state = trans->getState();
            }
            return state;
        }

    public:

        // void setOptions(const bool& is_auto, const int& default_action)
        // {
        //     _is_auto = is_auto;
        //     _default_action = default_action;
        // }

        // std::pair<bool,int> getOptions()
        // {
        //     auto pair = std::pair<bool,int>(_is_auto,_default_action);
        //     return pair;
        // }


    public:
        void setEnter(std::function<void()> enter)//感觉没啥必要
        {
            _enter = enter;
        }

        void setExec(std::function<std::pair<bool,int>()> exec)
        {
            _exec = exec;
        }

        void setExit(std::function<void()> exit)//同enter，真没啥必要，顶多去cout一下表示状态已经退出
        {
            _exit = exit;
        }

        void onEnter()//感觉没啥必要
        {
            if (_enter)
            {
                _enter();
            }
        }

        std::pair<bool,int> onExec()//TODO:让_exec返回int与bool（即pair），让状态自行决定是否自动运行
        {
            if (_exec)
            {
                auto tmp = _exec();
                return tmp;
            }

        }

        void onExit()//同enter，真没啥必要，顶多去cout一下表示状态已经退出
        {
            if (_exit)
            {
                _exit();
            }
        }
    };
    

    
}
