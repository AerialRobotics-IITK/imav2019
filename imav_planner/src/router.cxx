#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <imav_planner/task_info.h>
#include <detector_msgs/BBPoses.h>
#include <detector_msgs/BBPose.h>

std::string colour;
detector_msgs::BBPoses objects;
int id = 0;

void objCallback(const detector_msgs::BBPoses& msg)
{
    objects = msg;
}

void publishMsg(std::string mav_name, geometry_msgs::Point position, ros::Publisher *pubPtr, std::string loc_type)
{
    imav_planner::task_info msg;
    msg.header.stamp = ros::Time::now();
    msg.loc_type = loc_type;
    msg.mav_name = mav_name;
    msg.position = position;
    pubPtr->publish(msg);
    return;
}

void createTasks(detector_msgs::BBPoses *obj, ros::Publisher *pubPtr)
{
    for(int i=0; i < obj->object_poses.size(); i++)
    {
        if(obj->object_poses.at(i).type == id) publishMsg(obj->mav_name, obj->object_poses.at(i).position, pubPtr, "Drop");
    }
    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "router");
    ros::NodeHandle nh;

    nh.getParam("router/colour", colour);

    if(colour == "red") id = -10;
    else if(colour == "yellow") id = -20;
    else if(colour == "blue") id = -30;

    ros::Subscriber objSub = nh.subscribe("objects", 10, objCallback);
    ros::Publisher taskPub = nh.advertise<imav_planner::task_info>("task", 1);
    ros::Rate loopRate(10);

    while(ros::ok())
    {
        while(objects.imageID < 1) ros::spinOnce();
        createTasks(&objects, &taskPub);
        ros::spinOnce();
        loopRate.sleep();
    }
}