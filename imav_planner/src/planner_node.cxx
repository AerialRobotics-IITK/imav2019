#include<ros/ros.h>
#include<nav_msgs/Odometry.h>
#include<geometry_msgs/PoseStamped.h>

nav_msgs::Odometry magnus_odom;
nav_msgs::Odometry vega_odom;
nav_msgs::Odometry icarus_odom;

void Magnus_Callback(const nav_msgs::Odometry& msg)
{
    magnus_odom=msg;
}

void Vega_Callback(const nav_msgs::Odometry& msg)
{
    vega_odom=msg;
}

void Icarus_Callback(const nav_msgs::Odometry& msg)
{
    icarus_odom=msg;
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"planner");
    ros::NodeHandle nh;

    ros::Subscriber magnus_subscriber = nh.subscribe("magnus/ground_truth/odometry",10,Magnus_Callback);
    ros::Subscriber vega_subscriber = nh.subscribe("vega/ground_truth/odometry",10,Vega_Callback);
    ros::Subscriber icarus_subscriber = nh.subscribe("icarus/ground_truth/odometry",10,Icarus_Callback);

    ros::Publisher magnus_publisher = nh.advertise<geometry_msgs::PoseStamped>("magnus/command/pose",1);
    ros::Publisher vega_publisher = nh.advertise<geometry_msgs::PoseStamped>("vega/command/pose",1);
    ros::Publisher icarus_publisher = nh.advertise<geometry_msgs::PoseStamped>("icarus/command/pose",1);

    ros::Rate loopRate(10);

    while(ros::ok())
    {
        geometry_msgs::PoseStamped magnus_command;
        geometry_msgs::PoseStamped vega_command;
        geometry_msgs::PoseStamped icarus_command;

        magnus_publisher.publish(magnus_command);
        vega_publisher.publish(vega_command);
        icarus_publisher.publish(icarus_command);
        ros::spinOnce();
        loopRate.sleep();
    }
    
    return 0;
}