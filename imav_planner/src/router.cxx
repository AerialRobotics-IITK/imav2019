#include <ros/ros.h>

#include <std_msgs/String.h>
#include <sensor_msgs/NavSatFix.h>

#include <mav_utils_msgs/TaskInfo.h>
#include <mav_utils_msgs/RouterData.h>
#include <mav_utils_msgs/RouterInfo.h>
#include <mav_utils_msgs/BBPoses.h>

#define echo(x) std::cout << x << std::endl

std::string mavName, names[3];
std::string curr_state;
sensor_msgs::NavSatFix globalPose;
std::vector<mav_utils_msgs::RouterData> routerData[2];
mav_utils_msgs::BBPoses obj_data;
bool verbose = true;
int id=-1;

struct locData
{
    int drops;
    // bool publish;
    double latitude;
    double longitude;
};

int ids[3][8] = {{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0}};
struct locData objects[8];

void objCallback(const mav_utils_msgs::BBPoses& msg)
{
    obj_data = msg;
}

void r1Callback(const mav_utils_msgs::RouterInfo& msg)
{
    for(int i=0; i<8; i++)
    {
        ids[1][i] = msg.object_id.at(i);
    }
    routerData[0] = msg.router_data;
}

void r2Callback(const mav_utils_msgs::RouterInfo& msg)
{
    for(int i=0; i<8; i++)
    {
        ids[2][i] = msg.object_id.at(i);
    }
    routerData[1] = msg.router_data;
}

void poseCallback(const sensor_msgs::NavSatFix& msg)
{
    globalPose = msg;
}

void stateCallback(const std_msgs::String& msg)
{
    curr_state = msg.data;
}

void updateTable()
{
    int num, drops;
    for(int i=0; i<obj_data.object_poses.size(); i++)
    {
        num = drops = -1;
        if(obj_data.object_poses.at(i).store)
        {
            switch(obj_data.object_poses.at(i).type)
            {
                // house
                case 42: num = 0; drops = 1; break;

                // mailboxes
                case -10: num = 1; drops = 2; break;
                case -20: num = 2; drops = 2; break;
                case -30: num = 3; drops = 2; break;

                // lost packages
                case 10: num = 4; drops = 1; break;
                case 20: num = 5; drops = 1; break;
                case 30: num = 6; drops = 1; break;

                // crashed drone
                case 69: num = 7; drops = 1; break;
                default: continue;
            }

            if(ids[0][num]==0)
            {    
                ids[0][num] = 1; 
                objects[num].drops = drops; 
                // objects[num].publish = true;
                objects[num].latitude = globalPose.latitude;
                objects[num].longitude = globalPose.longitude;
            }
        }
    }

    for(int i=0; i<2; i++)
    {
        for(int j=0; j<routerData[i].size(); j++)
        {
            ids[0][routerData[i].at(j).id] = 1;

            struct locData data;
            data.drops = routerData[i].at(j).altitude;
            data.latitude = routerData[i].at(j).latitude;
            data.longitude = routerData[i].at(j).longitude;
            // data.publish = false;

            objects[routerData[i].at(j).id] = data;
        }
    }
}

void updateRouters(ros::Publisher *pub)
{
    mav_utils_msgs::RouterInfo info_msg;
    mav_utils_msgs::RouterData data;
    for(int i=0; i<8; i++)
    {
        info_msg.object_id.push_back(ids[0][i]);
        if(ids[0][i] == 1 && (ids[1][i] == 0 || ids[2][i] == 0))
        {
            data.id = i;
            data.altitude = (i>0 && i<4) ? 2 : 1;
            data.latitude = objects[i].latitude;
            data.longitude = objects[i].longitude;
            info_msg.router_data.push_back(data);
        }
    }
    info_msg.header.stamp = ros::Time::now();
    pub->publish(info_msg);
}

void publishTask(ros::Publisher *pub)
{
    if(curr_state != "Exploring") return;
    mav_utils_msgs::TaskInfo task;
    int num=-1;

    if(obj_data.object_poses.size() > 0)
    {
        task.header.stamp = ros::Time::now();
        task.is_local = true;
        task.position = obj_data.object_poses.at(0).position;
        if(obj_data.object_poses.at(0).type == -id*10)
        {
            task.loc_type = "Drop";
            num = id;
        }
        else 
        {
            task.loc_type = "Hover";
            switch(obj_data.object_poses.at(0).type)
            {
                // house
                case 42: num = 0;

                // mailboxes
                case -10: num = 1;
                case -20: num = 2;
                case -30: num = 3;

                // lost packages
                case 10: num = 4;
                case 20: num = 5;
                case 30: num = 6;

                // crashed drone
                case 69: num = 7;
            }
        }
        if(objects[num].drops > 0)
        {
            pub->publish(task);
            objects[num].drops -= 1;
        }
        return;
    }
    else if(objects[id].drops > 0)
    {
        task.header.stamp = ros::Time::now();
        task.is_local = false;
        task.latitude = objects[id].latitude;
        task.longitude = objects[id].longitude;
        task.altitude = 0;
        objects[id].drops -= 1;
        pub->publish(task);
        return;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "router");
    ros::NodeHandle ph("~");

    ros::Rate loopRate(10);
    
    ph.getParam("verbose", verbose);
    ph.getParam("mav_name", mavName);
    ph.getParam("names/red", names[0]);
    ph.getParam("names/yellow", names[1]);
    ph.getParam("names/blue", names[2]);

    for(int i=0; i<3; i++)
    {
        if(mavName == names[i]) id = i+1;
    }

    if(verbose) echo(id);

    ros::NodeHandle routers[3] = {"/" + names[id-1], "/" + names[(id)%3], "/" + names[(id+1)%3]};
    
    ros::Subscriber objSub = routers[0].subscribe("objects", 10, objCallback);
    ros::Subscriber stateSub = routers[0].subscribe("curr_state", 10, stateCallback);
    ros::Subscriber poseSub = routers[0].subscribe("global_pose", 10, poseCallback);
    ros::Subscriber subs[2] = {routers[1].subscribe("router/data", 10, r1Callback), routers[2].subscribe("router/data", 10, r2Callback)};
    
    ros::Publisher routerPub = ph.advertise<mav_utils_msgs::RouterInfo>("data", 10);
    ros::Publisher taskPub = routers[0].advertise<mav_utils_msgs::TaskInfo>("task", 10);
    
    while(ros::ok())
    {
        ros::spinOnce();
        updateTable();
        updateRouters(&routerPub);
        publishTask(&taskPub);
        loopRate.sleep();
    }    

}
