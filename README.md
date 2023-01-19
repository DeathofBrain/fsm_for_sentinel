# fsm_for_sentinel
为rm哨兵使用的状态机，参考开源通用状态机进行改写
- 增加了
1. 在状态运行结束后，尝试自动切换状态
2. 添加开关，控制是否自动切换

- TODO：
  1. 让_exec返回int与bool（即pair），让状态自行决定是否自动运行（在State->onExec函数上）*已完成*
     1. 优点：让状态自行决定，增强稳定性
     2. 缺点：每个exec函数必须含有pair返回值，略嫌麻烦（待改进）
  2. 解决exec必须有pair返回值的问题