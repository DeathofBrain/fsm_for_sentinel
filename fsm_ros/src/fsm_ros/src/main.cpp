#include "../include/fsm_ros/StateMachine.hpp"
#include "../include/fsm_ros/Actions.hpp"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <memory>

using namespace std;
using namespace GFSM;
using sptr = shared_ptr<State>;

void do_msg(const std_msgs::String::ConstPtr& msg_p, StateMachine& sm)
{
    ROS_INFO("%s",msg_p->data.c_str());
    sm.doAction(1);
}


int main(int argc, char *argv[])
{
    ros::init(argc,argv,"state_mac_node");
    ros::NodeHandle nh;
    ROS_INFO("HELLO");

    int Exit = 0;
    int Continue = 1;


    //init states
    StateMachine sm;
    sptr initState = make_shared<State>();
    sptr finalState = make_shared<State>();

    initState->setEnter([&]{
        ROS_INFO("我是状态1enter");
    });
    initState->setExec([&]{
        ROS_INFO("我是状态1exec");
        return make_pair(false,Continue);
    });
    initState->setExit([&]{
        ROS_INFO("我是状态1exit");
    });
    initState->addTransition(Continue,finalState);

    finalState->setEnter([&]{
        ROS_INFO("我是状态2enter");
    });
    finalState->setExec([&]{
        ROS_INFO("我是状态2exec");
        return make_pair(false,Continue);
    });
    finalState->setExit([&]{
        ROS_INFO("我是状态2exit");
    });
    finalState->addTransition(Continue,finalState);
    
    sm.addState(initState);
    sm.addState(finalState);

    sm.initState(initState);

    sm.start();


    ros::Subscriber sub = nh.subscribe<std_msgs::String>("state_mac_node",10,boost::bind(do_msg,_1,std::ref(sm)));
    ros::spin();
    return 0;
}
