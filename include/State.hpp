#pragma once
//状态类
#include "./Transition.hpp"
#include "./Action.hpp"

#include <string>
#include <vector>
#include <functional>
#include <memory>


namespace GFSM
{
    class State
    {
    private:
        std::string _name;
    private:
        std::vector<std::shared_ptr<Transition>> _trans;//此状态对应的信号——次态集合
        std::function<void()> _enter;
        std::function<void()> _exec;
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

        std::shared_ptr<State> doAction(const Action& e)
        {
            std::shared_ptr<Transition> trans;

            for (auto it : _trans)
            {
                if ((it)->getType() == e.type())
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
        void setEnter(std::function<void()> enter)
        {
            _enter = enter;
        }

        void setExec(std::function<void()> exec)
        {
            _exec = exec;
        }

        void setExit(std::function<void()> exit)
        {
            _exit = exit;
        }

        void onEnter()
        {
            if (_enter)
            {
                _enter();
            }
        }

        void onExec()
        {
            if (_exec)
            {
                _exec();
            }
        }

        void onExit()
        {
            if (_exit)
            {
                _exit();
            }
        }
    };
    

    
}
