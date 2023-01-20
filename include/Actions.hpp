#pragma once

namespace GFSM
{
    /**
     * @brief 定义信号
     * 
     */
    enum Actions
    {
        /**
         * @brief 提供给状态与状态机，表示无默认信号
         * 
         */
        NO_DEFAULT_ACTION = -1,
    

        OPEN = 0,
        CLOSE
    };
} // namespace GFSM
