#include <ros/ros.h>

#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>

#include <mav_utils_msgs/UTMPose.h>
#include <mav_utils_msgs/TaskInfo.h>
#include <mav_utils_msgs/RouterData.h>
#include <mav_utils_msgs/RouterInfo.h>
#include <mav_utils_msgs/BBPoses.h>

#define echo(x) std::cout << x << std::endl
#define numQuads 3
#define numObjects 8

std::string mavName, curr_state, names[numQuads];
std::vector<mav_utils_msgs::RouterData> routerData[numQuads - 1];
mav_utils_msgs::BBPoses obj_data;
mav_utils_msgs::UTMPose utm_pose;
nav_msgs::Odometry odom;
bool verbose = true;
bool received = false;
int id=-1, pubId = -1;

struct locData{
    int drops;
    double x;
    double y;
    // bool publish;
};

int ids[numQuads][numObjects];
struct locData objects[numObjects];
int types[numObjects] = {42, -10, -20, -30, 10, 20, 30, 69};
int quadTypes[numQuads];

void objCallback(const mav_utils_msgs::BBPoses& msg){obj_data = msg;}
void utmCallback(const mav_utils_msgs::UTMPose& msg){utm_pose = msg;}
void odomCallback(const nav_msgs::Odometry& msg){odom = msg;}
void stateCallback(const std_msgs::String& msg){curr_state = msg.data;}

void r1Callback(const mav_utils_msgs::RouterInfo& msg){
    for(int i=0; i<numObjects; i++) ids[1][i] = msg.object_id.at(i);
    routerData[0] = msg.router_data;
}
void r2Callback(const mav_utils_msgs::RouterInfo& msg){
    for(int i=0; i<numObjects; i++) ids[2][i] = msg.object_id.at(i);
    routerData[1] = msg.router_data;
}

void updateTable(){
    // if(verbose) echo("Updating table");

    int num, drops;
    // if(verbose) echo(" Processing detector data");
    for(int i=0; i<obj_data.object_poses.size(); i++){
        num = drops = -1;
        if(obj_data.object_poses.at(i).store){
            switch(obj_data.object_poses.at(i).type){
                // house
                case 42: num = 0; drops = 1; break;

                // mailboxes
                case -10: num = 1; drops = 1; break;
                case -20: num = 2; drops = 1; break;
                case -30: num = 3; drops = 1; break;

                // lost packages
                case 10: num = 4; drops = 1; break;
                case 20: num = 5; drops = 1; break;
                case 30: num = 6; drops = 1; break;

                // crashed drone
                case 69: num = 7; drops = 1; break;
                default: continue;
            }

            if(ids[0][num]==0){    
                ids[0][num] = 1; 
                objects[num].drops = drops; 
                // objects[num].publish = true;
                objects[num].x = obj_data.object_poses.at(i).position.x - odom.pose.pose.position.x + utm_pose.pose.position.x; 
                objects[num].y = obj_data.object_poses.at(i).position.y - odom.pose.pose.position.y + utm_pose.pose.position.y;
            }

            if(verbose) echo("  Stored from detector x = " << objects[num].x << ", y = " << objects[num].y << ", drops = " << drops << " at array pos = " << num);
        }
    }

    // if(verbose) echo(" Processing data from other routers");
    for(int i=0; i<numQuads - 1; i++){
        for(int j=0; j<routerData[i].size(); j++){
            if(ids[0][routerData[i].at(j).id] == 0){
                ids[0][routerData[i].at(j).id] = 1;
    
                struct locData data;
                data.drops = routerData[i].at(j).position.z;
                data.x = routerData[i].at(j).position.x;
                data.y = routerData[i].at(j).position.y;
                // data.publish = false;
    
                if(routerData[i].at(j).id == id){
                    received = true;
                    data.drops = 1;
                }
                objects[routerData[i].at(j).id] = data;
    
                if(verbose) echo("  Stored from table x = " << data.x << ", y = " << data.y << ", drops = " << data.drops << " at array pos = " << routerData[i].at(j).id);
            }
        }
    }

    return;
}

void updateRouters(ros::Publisher *pub){
    // if(verbose) echo("Updating routers");

    mav_utils_msgs::RouterInfo info_msg;
    mav_utils_msgs::RouterData data;

    for(int i=0; i<numObjects; i++){
        info_msg.object_id.push_back(ids[0][i]);
        if(ids[0][i] == 1 && (ids[1][i] == 0 || ids[2][i] == 0)){
            if(verbose) echo(" Publishing to other routers");
            data.id = i;
            data.position.z = 0; 
            data.position.x = objects[i].x;
            data.position.y = objects[i].y;
            info_msg.router_data.push_back(data);
            if(verbose) echo("  Published x = " << data.position.x << ", y = " << data.position.y << ", drops = " << data.position.z << " for array pos = " << data.id);
        } 
    }

    info_msg.header.stamp = ros::Time::now();
    pub->publish(info_msg);

    return;
}

void publishTask(ros::Publisher *pub){
    if(curr_state == "Drop") objects[pubId].drops -= 1;

    if(curr_state != "Exploring") return;
    // if(verbose) echo("Publishing tasks");

    mav_utils_msgs::TaskInfo task;
    int num=-1;

    if(obj_data.object_poses.size() > 0){
        // if(verbose) echo(" Processing detector data");
        task.header.stamp = ros::Time::now();
        task.position.x = obj_data.object_poses.at(0).position.x;
        task.position.y = obj_data.object_poses.at(0).position.y;
        task.position.z = 0;
        task.id = obj_data.object_poses.at(0).type;
        if(task.id == -id*10){
            task.loc_type = "Drop";
            num = id;
        }
        else{
            task.loc_type = "Hover";
            switch(task.id){
                // house
                case 42: num = 0; break;

                // mailboxes
                case -10: num = 1; break;
                case -20: num = 2; break;
                case -30: num = 3; break;

                // lost packages
                case 10: num = 4; break;
                case 20: num = 5; break;
                case 30: num = 6; break;

                // crashed drone
                case 69: num = 7; break;
            }
        }
        if(objects[num].drops > 0){
            pub->publish(task);
            pubId = num;
            if(verbose) echo("  Published from detector: loc_type = " << task.loc_type << ", position: x = " << task.position.x << ", y = " << task.position.y << ", z = " << task.position.z);
            return;
        }
    }
    
    if(objects[id].drops > 0 && received){
        // if(verbose) echo(" Publishing from table");
        task.header.stamp = ros::Time::now();
        task.loc_type = "Drop";
        task.id = -id*10;
        task.position.x = objects[id].x + odom.pose.pose.position.x - utm_pose.pose.position.x;
        task.position.y = objects[id].y + odom.pose.pose.position.y - utm_pose.pose.position.y;
        task.position.z = 0;
        pub->publish(task);
        pubId = id;
        echo("  Published from table: loc_type = " << task.loc_type << ", position: x = " << task.position.x << ", y = " << task.position.y << ", z = " << task.position.z);
    }

    return;
}

int main(int argc, char** argv){

    ros::init(argc, argv, "router");
    ros::NodeHandle ph("~");

    ros::Rate loopRate(10);
    
    ph.getParam("verbose", verbose);
    ph.getParam("mav_name", mavName);

    // to generalie change to list
    ph.getParam("names/red", names[0]);
    ph.getParam("names/yellow", names[1]);
    ph.getParam("names/blue", names[2]);

    for(int i=0; i<numQuads; i++)  if(mavName == names[i]) id = i+1;
    if(verbose) echo(id);

    for(int i = 0; i < numQuads; i++){
        for(int j=0; j < numObjects; j++){
            ids[i][j] = 0;
        }
    }

    for(int i = 0; i < numObjects; i++) objects[i].drops = 1;

    ros::NodeHandle routers[numQuads] = {"/" + names[id-1], "/" + names[(id)%numQuads], "/" + names[(id+1)%numQuads]};
    
    ros::Subscriber objSub = routers[0].subscribe("objects", 10, objCallback);
    ros::Subscriber stateSub = routers[0].subscribe("curr_state", 10, stateCallback);
    ros::Subscriber utmSub = routers[0].subscribe("utm_pose", 10, utmCallback);
    ros::Subscriber odomSub = routers[0].subscribe("odometry", 10, odomCallback);
    ros::Subscriber subs[numQuads - 1] = {routers[1].subscribe("router/data", 10, r1Callback), routers[2].subscribe("router/data", 10, r2Callback)};
    
    ros::Publisher routerPub = ph.advertise<mav_utils_msgs::RouterInfo>("data", 10);
    ros::Publisher taskPub = routers[0].advertise<mav_utils_msgs::TaskInfo>("task", 10);
    
    while(ros::ok()){
        ros::spinOnce();
        updateTable();
        updateRouters(&routerPub);
        publishTask(&taskPub);
        loopRate.sleep();
    }    

    return 0;
}
