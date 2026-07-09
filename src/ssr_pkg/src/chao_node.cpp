#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"chao_node");
    printf("吃了吗？我来了!\n");

    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::String>("kuai_shang_che_kai_hei",10);

    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        printf("我要刷屏了！\n");
        std_msgs::String msgs;
        msgs.data = "国服马超,带飞";
        pub.publish(msgs);
        loop_rate.sleep();
    } 
    return 0;
}
 
 
 