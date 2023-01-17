#pragma once
//信号类
namespace GFSM
{
    class Action
    {
    private:
        int _type;//对应enum类中信号
    public:
        Action() = delete;
        Action(const int& type):_type(type){}
        ~Action() = default;
    public:
        int type() const//返回标志位
        {
            return _type;
        }

        void setType(const int& type)//设置标志位
        {
            _type = type;
        }
    };
} // namespace GFSM
