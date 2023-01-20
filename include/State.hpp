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
    private:
        std::vector<std::shared_ptr<Transition>> _trans;//此状态对应的信号——次态集合
        std::function<void()> _enter;
        std::function<std::pair<bool,int>()> _exec;
        std::function<void()> _exec_no_return;
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
        void setEnter(std::function<void()> enter)//感觉没啥必要
        {
            _enter = enter;
        }

        void setExec(std::function<std::pair<bool,int>()> exec)
        {

            if (_exec_no_return)
            {
                cout<<"不可重复定义exec函数"<<endl;
                return;
            }
            
            _exec = exec;
        }

        void setExecNoReturn(std::function<void()> exec)
        {

            if (_exec)
            {
                cout<<"不可重复定义exec函数"<<endl;
                return;
            }


            _exec_no_return = exec;
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

            if (_exec)//含有返回值
            {
                auto tmp = _exec();
                return tmp;
            }

            if (_exec_no_return)//不含返回值
            {
                _exec_no_return();
                auto tmp = make_pair(false,Actions::NO_DEFAULT_ACTION);
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
