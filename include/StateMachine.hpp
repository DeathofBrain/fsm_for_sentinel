#pragma once

#include "State.hpp"

namespace GFSM
{
    class StateMachine
    {
    private:
        std::vector<std::shared_ptr<State>> _states;
        std::shared_ptr<State> _currentState;
    public:
        StateMachine() = default;
        ~StateMachine() = default;
    public:
        void addState(std::shared_ptr<State> state)
        {
            if (state)
            {
                _states.push_back(state);
            }
            
        }

        void initState(std::shared_ptr<State> state)
        {
            _currentState = state;
        }

        bool start()
        {
            if(!_currentState)
            {
                return false;
            }
            _currentState->onEnter();
            _currentState->onExec();
            return true;
        }

        void doAction(const Action& e)
        {
            auto state = _currentState->doAction(e);
            if(!state)
            {
                return;
            }
            if (_currentState != state)
            {
                _currentState->onExit();
                _currentState = state;
                _currentState->onEnter();
            }
            _currentState->onExec();
            return;
        }
    };
    

    
} // namespace GFSM
