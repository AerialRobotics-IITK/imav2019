#include<ros/ros.h>
#include<nav_msgs/Odometry.h>
#include<geometry_msgs/PoseStamped.h>
#include<std_msgs/Bool.h>

nav_msgs::Odometry mav_odom;

void Magnus_Callback(const nav_msgs::Odometry& msg)
{
    mav_odom=msg;
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"planner");
    ros::NodeHandle nh;

    ros::Subscriber magnus_subscriber = nh.subscribe("ground_truth/odometry",10,Magnus_Callback);

    ros::Publisher magnus_publisher = nh.advertise<geometry_msgs::PoseStamped>("command/pose", 1);
    ros::Publisher gripper_pub = nh.advertise<std_msgs::Bool>("gripper_status", 1);

    ros::Rate loopRate(10);

    while(ros::ok())
    {
        geometry_msgs::PoseStamped mav_cmd_pose_;
        std_msgs::Bool gripper_status_;

        magnus_publisher.publish(mav_cmd_pose_);
        ros::spinOnce();
        loopRate.sleep();
    }
    
    return 0;
}