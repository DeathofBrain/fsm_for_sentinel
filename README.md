# fsm_for_sentinel
为rm哨兵使用的状态机，参考开源通用状态机进行改写
- 增加了
1. 在状态运行结束后，尝试自动切换状态
2. 添加开关，控制是否自动切换
3. 添加可由外部定义下一状态的api
4. 让_exec返回int与bool（即pair）或void，让状态自行决定是否自动运行（在State->onExec函数上）

