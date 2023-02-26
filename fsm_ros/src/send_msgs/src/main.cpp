#include "ros/ros.h"
#include "std_msgs/String.h"


int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");

    ros::init(argc,argv,"send_msg_node");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::String>("state_mac_node",10);
    std_msgs::String msg;
    msg.data = "Go";
    while (ros::ok())
    {
        std::cin.get();
        pub.publish(msg);
        ROS_INFO("已发送");
        ros::spinOnce();
    }
    
    return 0;
}
