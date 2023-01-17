#include <iostream>
#include "../include/GeneralFSM.hpp"

using namespace std;
using namespace GFSM;
/*
    测试代码源自https://toscode.gitee.com/andwp/cpp-state/blob/master/ConsoleApplication1/ConsoleApplication1.cpp
*/
int main()
{
    std::cout << "Hello World!\n";
    int Exit = 0; // 退出。 
    int Continue = 1; // 继续执行。

    StateManager fsm;

    State* yellowToRedState = new State;  // （转红灯的）黄灯状态。
    State* greenState = new State;  // 绿灯状态。
    State* yellowToGreenState = new State;  // （转绿灯的）黄灯状态。
    State* redState = new State;  // 红灯状态。
    State* initState = new State;  // 初始状态。
    State* finalState = new State;  // 关闭状态。

    auto finalExec = []() {cout << "终止程序！" << endl; };
    finalState->setExec(finalExec);

    yellowToRedState->setEnter([]() {cout << "进入黄灯状态。" << endl; });
    yellowToRedState->setExec([]() {cout << "Yellow (To Red ) Light Exec" << endl; });
    yellowToRedState->setQuit([]() {cout << "退出黄灯状态。" << endl; });
    yellowToRedState->addTransition(Continue, redState); //黄灯关闭，转为红灯。
    yellowToRedState->addTransition(Exit, finalState); // 直接关闭，转为终止。

    greenState->setEnter([]() { cout << "进入绿灯状态" << endl; });
    greenState->setExec([]() { cout << "Green Light Exec" << endl; });
    greenState->setQuit([]() { cout << "退出绿灯状态" << endl; });
    greenState->addTransition(Continue, yellowToRedState);
    greenState->addTransition(Exit, finalState); // 直接关闭，转为终止。

    yellowToGreenState->setEnter([]() { cout << "进入黄灯状态" << endl; });
    yellowToGreenState->setExec([]() { cout << "Yellow (ToGreen ) Light Exec" << endl; });
    yellowToGreenState->setQuit([]() { cout << "退出黄灯状态" << endl; });
    yellowToGreenState->addTransition(Continue, greenState); //黄灯关闭，转为绿灯。
    yellowToGreenState->addTransition(Exit, finalState); // 直接关闭，转为终止。

    redState->setEnter([]() { cout << "进入红灯状态" << endl; });
    redState->setExec([]() { cout << "Red Light Exec" << endl; });
    redState->setQuit([]() { cout << "退出红灯状态" << endl; });
    redState->addTransition(Continue, yellowToGreenState); // 红灯关闭，黄灯打开。
    redState->addTransition(Exit, finalState); // 直接关闭，转为终止。
     
    initState->setExec([]() { cout << "InitState Exec" << endl; });
    initState->addTransition(Continue, redState); // 初始后，切换为红灯。
    initState->addTransition(Exit, finalState); // 直接关闭，转为终止。

    fsm.addState(yellowToRedState);
    fsm.addState(greenState);
    fsm.addState(yellowToGreenState);
    fsm.addState(redState);
    fsm.addState(finalState);
    fsm.addState(initState);

    fsm.setInitState(initState);

    if (fsm.start())
    {
        std::cout << "启动完毕。" << endl;
        // 切换到下一个。
        fsm.doAction(Action(Continue)); 
        fsm.doAction(Action(Continue)); 
        fsm.doAction(Action(Continue));
        fsm.doAction(Action(Continue));
        fsm.doAction(Action(Continue));
        fsm.doAction(Action(Continue));
        fsm.doAction(Action(Continue));
        fsm.doAction(Action(Continue));
        fsm.doAction(Action(Continue));
        fsm.doAction(Action(Continue));
        // 切换到终止。
        fsm.doAction(Action(Exit));
    }
    else
    {
        std::cout << " 启动状态机失败。";
    }
    return 0;
}