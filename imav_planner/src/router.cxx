#include <ros/ros.h>
#include <imav_planner/task_info.h>
#include <detector_msgs/BBPoses.h>
#include <detector_msgs/BBPose.h>

std::string mav_name;
std::string names[3];
detector_msgs::BBPoses objects[3];
int id = 0, idIndex = 0;
int types[3] = {-10,-20,-30};

void redCallback(const detector_msgs::BBPoses& msg)
{
    objects[0] = msg;
}

void yellowCallback(const detector_msgs::BBPoses& msg)
{
    objects[1] = msg;
}

void blueCallback(const detector_msgs::BBPoses& msg)
{
    objects[2] = msg;
}

void publishMsg(std::string mav_name, geometry_msgs::Point position, ros::Publisher *pubPtr)
{
    imav_planner::task_info msg;
    msg.header.stamp = ros::Time::now();
    msg.loc_type = "Drop";
    msg.mavName = mav_name;
    msg.position = position;
    pubPtr->publish(msg);
    return;
}

void createTasks(std::string mav_name, detector_msgs::BBPoses *obj, ros::Publisher *pubPtr)
{
    for(int i=0; i<obj->object_poses.size(); i++)
    {
        detector_msgs::BBPose pose = obj->object_poses.at(i);
        if(pose.type == -id) publishMsg(mav_name, pose.position, pubPtr);
    }
    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "router");
    ros::NodeHandle nh;

    nh.getParam("router/mavName", mav_name);
    nh.getParam("router/redName", names[0]);
    nh.getParam("router/yellowName", names[1]);
    nh.getParam("router/blueName", names[2]);

    for(int i=0; i<3; i++)
    {
        if(names[i] == mav_name) 
        {
            id = -types[i];
            i = idIndex;
            break;
        }
    }    

    ros::Subscriber redSub = nh.subscribe("red", 10, redCallback);
    ros::Subscriber yellowSub = nh.subscribe("yellow", 10, yellowCallback);
    ros::Subscriber blueSub = nh.subscribe("blue", 10, blueCallback);

    ros::Publisher taskPub = nh.advertise<imav_planner::task_info>("task", 1);
    ros::Rate loopRate(10);

    while(ros::ok())
    {
        while(objects[0].imageID < 1 && objects[1].imageID < 1 && objects[2].imageID < 1) ros::spinOnce();
        for(int i=0; i<3; i++) createTasks(names[(idIndex+i)%3], objects + (idIndex+i)%3, &taskPub);
        ros::spinOnce();
        loopRate.sleep();
    }
}