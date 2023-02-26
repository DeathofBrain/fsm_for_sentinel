#include "../include/fsm_ros/StateMachine.hpp"
#include "../include/fsm_ros/Actions.hpp"
#include "ros/ros.h"
#include <memory>

using namespace std;
using namespace GFSM;

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"test_node");
    ROS_INFO("HELLO");

    int Exit = 0;
    int Continue = 1;

    StateMachine sm;

    using sptr = shared_ptr<State>;
    sptr initState = make_shared<State>();
    sptr finalState = make_shared<State>();
    sptr openState = make_shared<State>();
    sptr closeState = make_shared<State>();

    auto finalExec = [](){
            cout<<"程序终止"<<endl;
        };
    finalState->setExecNoReturn(finalExec);

    initState->setExec([&]{
            cout<<"初始化"<<endl;
            pair<bool,int> ret = make_pair(true,Continue);
            return ret;
        });
    initState->addTransition(Continue,openState);
    initState->addTransition(Exit,closeState);


    openState->setEnter([]{cout<<"灯正在打开"<<endl;});
    openState->setExec([&]{
            cout<<"灯已经打开"<<endl;
            pair<bool,int> ret = make_pair(true,Continue);
            return ret;            
        });
    openState->setExit([]{cout<<"有人关闭了开关"<<endl;});
    openState->addTransition(Continue,closeState);
    openState->addTransition(Exit,finalState);


    closeState->setEnter([]{cout<<"灯正在关闭"<<endl;});
    closeState->setExecNoReturn([&]{
            cout<<"灯已经关闭"<<endl;
        });
    closeState->setExit([]{cout<<"有人打开了开关"<<endl;});
    closeState->addTransition(Continue,openState);
    closeState->addTransition(Exit,finalState);

    sm.addState(initState);
    sm.addState(finalState);
    sm.addState(openState);
    sm.addState(closeState);

    sm.initState(initState);

    sm.start();
    // 切换到下一个。
    //sm.doAction(Continue);

    // 切换到终止。
    sm.doAction(Exit);
    return 0;
}
