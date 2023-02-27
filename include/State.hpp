#pragma once
/**
 * @file State.hpp
 * @brief 状态类
 * @copyright Copyright (c) 2023
 */
#include "./Transition.hpp"
#include "./Actions.hpp"


#include <iostream>
#include <string>
#include <vector>
#include <functional>
#include <memory>


/// @brief 
namespace GFSM
{
    /**
     * @brief 状态类
     * 
     */
    class State
    {
    private:
        /**
         * @brief 用于后期确认状态是否符合预期
         * 
         */
        std::string _name;
    private:
        /**
         * @brief 此状态对应的信号——次态集合
         * 
         */
        std::vector<std::shared_ptr<Transition>> _trans;

        /**
         * @brief 进入状态时的回调函数
         * 
         */
        std::function<void()> _enter;

        /**
         * @brief 含options的状态中回调函数
         * 
         */
        std::function<std::pair<bool,int>()> _exec;

        /**
         * @brief 不含options的状态中回调函数
         * 
         */
        std::function<void()> _exec_no_return;

        /**
         * @brief 退出状态时调用的回调函数
         * 
         */
        std::function<void()> _exit;
    public:
        State() = default;
        /**
         * @brief 用于后期确认状态是否符合预期
         * 
         * @param name 状态名称
         */
        State(const std::string& name):_name(name){}
        ~State() = default;

        /**
         * @brief 获取状态名字
         * 
         * @return std::string 返回名称
         */
        std::string getName() const
        {
            return _name;
        }
    public:
        /**
         * @brief 添加信号——次态关系
         * 
         * @param type 信号(在Actions中定义)
         * @param dest 信号对应次态
         */
        void addTransition(const int& type, std::shared_ptr<State> dest)
        {
            std::shared_ptr<Transition> transition = std::make_shared<Transition>(type,dest);
            _trans.push_back(transition);
        }
        /**
         * @brief 提供给状态机的api函数，用于找出信号e下的次态并返回给状态机
         * 
         * @param e 信号
         * @return std::shared_ptr<State> 次态(指针)
         */
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

        /**
         * @brief 设置含有options返回值的回调函数
         * 
         * @param exec pair<bool,int>型函数指针
         */
        void setExec(std::function<std::pair<bool,int>()> exec)
        {

            if (_exec_no_return)//检测是否重复定义exec函数
            {
                std::cout<<"不可重复定义exec函数"<<std::endl;
                return;
            }
            
            _exec = exec;
        }

        /**
         * @brief 设置不含有options返回值的回调函数
         * 
         * @param exec void型函数指针
         */
        void setExecNoReturn(std::function<void()> exec)
        {

            if (_exec)//检测是否重复定义exec函数
            {
                std::cout<<"不可重复定义exec函数"<<std::endl;
                return;
            }


            _exec_no_return = exec;
        }
        
        void setExit(std::function<void()> exit)//同enter，真没啥必要，顶多去cout一下表示状态已经退出
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
                auto tmp = std::make_pair(false,Actions::NO_DEFAULT_ACTION);
                return tmp;
            }
        }

        void onExit()
        {
            if (_exit)
            {
                _exit();
            }
        }
    public:
        /**
         * @brief 初始化状态函数
         * 
         * @param enter 进入状态时的回调函数
         * @param exec 保持状态时的回调函数（pair型）
         * @param exit 退出状态时的回调函数
         */
        void initState(std::function<void()> enter,std::function<std::pair<bool,int>()> exec,std::function<void()> exit)
        {
            this->setEnter(enter);
            this->setExec(exec);
            this->setExit(exit);
        }
    };
    

    
}
