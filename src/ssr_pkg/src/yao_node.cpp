#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"yao_node");
    printf("顶级厨师我TM来了!\n");

    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::String>("gie_gie_kuai_dai_wo",10);

    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        printf("我要刷屏了！\n");
        std_msgs::String msgs;
        msgs.data = "求上分+V";
        pub.publish(msgs);
        loop_rate.sleep();
    } 
    return 0;
}
 
 
 