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
        std::function<void()> _enter;//进入状态回调函数
        std::function<std::pair<bool,int>()> _exec;//含options的状态中回调函数
        std::function<void()> _exec_no_return;//不含options的状态中回调函数
        std::function<void()> _exit;//退出状态时调用的回调函数
    public:
        State() = default;
        State(const std::string& name):_name(name){}//用于后期确认状态是否符合预期
        ~State() = default;
        std::string getName() const
        {
            return _name;
        }
    public:
        void addTransition(const int& type, std::shared_ptr<State> dest)//添加信号——次态关系
        {
            std::shared_ptr<Transition> transition = std::make_shared<Transition>(type,dest);
            _trans.push_back(transition);
        }

        std::shared_ptr<State> doAction(const int& e)//提供给状态机的api函数，用于找出信号e下的次态并返回给状态机
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

    public://set，get环节
        void setEnter(std::function<void()> enter)//感觉没啥必要
        {
            _enter = enter;
        }

        void setExec(std::function<std::pair<bool,int>()> exec)//含有返回值的回调函数
        {

            if (_exec_no_return)//检测是否重复定义exec函数
            {
                cout<<"不可重复定义exec函数"<<endl;
                return;
            }
            
            _exec = exec;
        }

        void setExecNoReturn(std::function<void()> exec)
        {

            if (_exec)//检测是否重复定义exec函数
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

        std::pair<bool,int> onExec()
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
