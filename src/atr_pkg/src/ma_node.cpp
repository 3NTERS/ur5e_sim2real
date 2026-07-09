#include <ros/ros.h>
#include <std_msgs/String.h>

void chao_callback(std_msgs::String msg)
{
    ROS_INFO(msg.data.c_str() );
    
}
void yao_callback(std_msgs::String msg)
{
    ROS_WARN(msg.data.c_str() );
    
}

int main(int argc, char  *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"ma_node");

    ros::NodeHandle nh;
    ros::Subscriber sub =nh.subscribe("kuai_shang_che_kai_hei",10,chao_callback);
    ros::Subscriber sub_2 =nh.subscribe("gie_gie_kuai_dai_wo",10,yao_callback);


    while(ros::ok())
    {
        ros::spinOnce();//回头看接收新消息
    }

    return 0;
}