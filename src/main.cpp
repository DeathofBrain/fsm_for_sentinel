#include "../include/StateMachine.hpp"
#include "../include/Actions.hpp"
#include <memory>
#include <iostream>

using namespace std;
using namespace GFSM;

int main(int argc, char const *argv[])
{
    
    int Exit = 0;
    int Continue = 1;

    StateMachine sm;

    using sptr = shared_ptr<State>;
    sptr initState = make_shared<State>();
    sptr finalState = make_shared<State>();
    sptr openState = make_shared<State>();
    sptr closeState = make_shared<State>();

    auto finalExec = [](){cout<<"程序终止"<<endl;};
    finalState->setExec(finalExec);

    initState->setExec([]{cout<<"初始化"<<endl;});
    initState->addTransition(Continue,openState);
    initState->addTransition(Exit,closeState);

    openState->setEnter([]{cout<<"灯正在打开"<<endl;});
    openState->setExec([]{cout<<"灯已经打开"<<endl;});
    openState->setExit([]{cout<<"有人关闭了开关"<<endl;});
    openState->addTransition(Continue,closeState);
    openState->addTransition(Exit,closeState);

    closeState->setEnter([]{cout<<"灯正在关闭"<<endl;});
    closeState->setExec([]{cout<<"灯已经关闭"<<endl;});
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
    sm.doAction(Continue); 
    sm.doAction(Continue); 
    sm.doAction(Continue);
    sm.doAction(Continue);
    sm.doAction(Continue);
    sm.doAction(Continue);
    sm.doAction(Continue);
    sm.doAction(Continue);
    sm.doAction(Continue);
    sm.doAction(Continue);
    // 切换到终止。
    sm.doAction(Exit);
    

    //system("pause");
    return 0;
}
