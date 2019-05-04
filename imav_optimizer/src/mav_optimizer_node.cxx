#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/SetModelState.h>
#include <std_msgs/Bool.h>

using namespace std;

nav_msgs::Odometry magnus_odom_;
nav_msgs::Odometry vega_odom_;
nav_msgs::Odometry icarus_odom_;

geometry_msgs::Pose red_package_0_pose, red_package_1_pose, red_mailbox_pose,
    yellow_package_0_pose, yellow_package_1_pose, yellow_mailbox_pose,
    blue_package_0_pose, blue_package_1_pose ,blue_mailbox_pose;

void magnus_cb(const nav_msgs::Odometry &msg)
{
    magnus_odom_ = msg;
}

void vega_cb(const nav_msgs::Odometry &msg)
{
    vega_odom_ = msg;
}

void icarus_cb(const nav_msgs::Odometry &msg)
{
    icarus_odom_ = msg;
}

void obj_cb(const gazebo_msgs::ModelStates &msg)
{
    for (int i_ = 0; i_ < (msg.name.size()); i_++)
    {
        string obj_name = msg.name[i_];
        if (obj_name == "Blue_Package_0")
            blue_package_0_pose = msg.pose[i_];
        if (obj_name == "Blue_Package_1")
            blue_package_1_pose = msg.pose[i_];
        if (obj_name == "Yellow_Package_0")
            yellow_package_0_pose = msg.pose[i_];
        if (obj_name == "Yellow_Package_1")
            yellow_package_1_pose = msg.pose[i_];
        if (obj_name == "Red_Package_0")
            red_package_0_pose = msg.pose[i_];
        if (obj_name == "Red_Package_1")
            red_package_1_pose = msg.pose[i_];
        if (obj_name == "Red_Mailbox")
            red_mailbox_pose = msg.pose[i_];
        if (obj_name == "Yellow_Mailbox")
            yellow_mailbox_pose = msg.pose[i_];
        if (obj_name == "Blue_Mailbox")
            blue_mailbox_pose = msg.pose[i_];
    }
}

float dist_bw_obj_mav_(geometry_msgs::Pose obj_pose, geometry_msgs::Pose mav_pose_)
{
    return sqrt(pow((obj_pose.position.x - mav_pose_.position.x), 2) + pow((obj_pose.position.y - mav_pose_.position.y), 2) + pow((obj_pose.position.z - mav_pose_.position.z), 2));
}

float dist_bw_obj_mailbox_(geometry_msgs::Pose obj_pose, geometry_msgs::Pose mailbox_pose_)
{
    return sqrt(pow((obj_pose.position.x - mailbox_pose_.position.x), 2) + pow((obj_pose.position.y - mailbox_pose_.position.y), 2) + pow((obj_pose.position.z - mailbox_pose_.position.z), 2));
}

int min_index_(vector<float> vec_)
{
    auto min_ = min_element(begin(vec_), end(vec_));
    return distance(begin(vec_), min_);
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "optimizer");
    ros::NodeHandle nh;

    ros::Subscriber magnus_sub_ = nh.subscribe("/magnus/ground_truth/odometry", 100, magnus_cb);
    ros::Subscriber vega_sub_ = nh.subscribe("/vega/ground_truth/odometry", 100, vega_cb);
    ros::Subscriber icarus_sub_ = nh.subscribe("/icarus/ground_truth/odometry", 100, icarus_cb);
    ros::Subscriber obj_sub_ = nh.subscribe("/gazebo/model_states", 100, obj_cb);
    ros::Publisher magnus_optimized = nh.advertise<std_msgs::String>("/magnus/optimized",1);
    ros::Publisher vega_optimized = nh.advertise<std_msgs::String>("/vega/optimized", 1);
    ros::Publisher icarus_optimized = nh.advertise<std_msgs::String>("/icarus/optimized", 1);

    ros::Rate loop_rate(100);

    vector<float> magnus_list;
    vector<float> vega_list;
    vector<float> icarus_list;
    string magnus_package_name;
    string vega_package_name;
    string icarus_package_name;

    while (ros::ok())
    {   
        //Current Idea: To pick object having min(dist_from_quad+dist_from_obj_to_mailbox)
        //Magnus_Optimizer
        magnus_list.push_back(dist_bw_obj_mav_(blue_package_0_pose, magnus_odom_.pose.pose) + dist_bw_obj_mailbox_(blue_package_0_pose, blue_mailbox_pose));
        magnus_list.push_back(dist_bw_obj_mav_(blue_package_1_pose, magnus_odom_.pose.pose) + dist_bw_obj_mailbox_(blue_package_1_pose, blue_mailbox_pose));
        magnus_list.push_back(dist_bw_obj_mav_(red_package_0_pose, magnus_odom_.pose.pose) + dist_bw_obj_mailbox_(red_package_0_pose, red_mailbox_pose));
        magnus_list.push_back(dist_bw_obj_mav_(red_package_1_pose, magnus_odom_.pose.pose) + dist_bw_obj_mailbox_(red_package_1_pose, red_mailbox_pose));
        magnus_list.push_back(dist_bw_obj_mav_(yellow_package_0_pose, magnus_odom_.pose.pose) + dist_bw_obj_mailbox_(yellow_package_0_pose, yellow_mailbox_pose));
        magnus_list.push_back(dist_bw_obj_mav_(yellow_package_1_pose, magnus_odom_.pose.pose) + dist_bw_obj_mailbox_(yellow_package_1_pose, yellow_mailbox_pose));

        if (min_index_(magnus_list) == 0)
            magnus_package_name = "Blue_Package_0";
        else if (min_index_(magnus_list) == 1)
            magnus_package_name = "Blue_Package_1";
        else if (min_index_(magnus_list) == 2)
            magnus_package_name = "Red_Package_0";
        else if (min_index_(magnus_list) == 3)
            magnus_package_name = "Red_Package_1";
        else if (min_index_(magnus_list) == 4)
            magnus_package_name = "Yellow_Package_0";
        else if (min_index_(magnus_list) == 5)
            magnus_package_name = "Yellow_Package_1";

        magnus_optimized.publish(magnus_package_name);

        //vega_Optimizer
        vega_list.push_back(dist_bw_obj_mav_(blue_package_0_pose, vega_odom_.pose.pose) + dist_bw_obj_mailbox_(blue_package_0_pose, blue_mailbox_pose));
        vega_list.push_back(dist_bw_obj_mav_(blue_package_1_pose, vega_odom_.pose.pose) + dist_bw_obj_mailbox_(blue_package_1_pose, blue_mailbox_pose));
        vega_list.push_back(dist_bw_obj_mav_(red_package_0_pose, vega_odom_.pose.pose) + dist_bw_obj_mailbox_(red_package_0_pose, red_mailbox_pose));
        vega_list.push_back(dist_bw_obj_mav_(red_package_1_pose, vega_odom_.pose.pose) + dist_bw_obj_mailbox_(red_package_1_pose, red_mailbox_pose));
        vega_list.push_back(dist_bw_obj_mav_(yellow_package_0_pose, vega_odom_.pose.pose) + dist_bw_obj_mailbox_(yellow_package_0_pose, yellow_mailbox_pose));
        vega_list.push_back(dist_bw_obj_mav_(yellow_package_1_pose, vega_odom_.pose.pose) + dist_bw_obj_mailbox_(yellow_package_1_pose, yellow_mailbox_pose));

        if (min_index_(vega_list) == 0)
            vega_package_name = "Blue_Package_0";
        else if (min_index_(vega_list) == 1)
            vega_package_name = "Blue_Package_1";
        else if (min_index_(vega_list) == 2)
            vega_package_name = "Red_Package_0";
        else if (min_index_(vega_list) == 3)
            vega_package_name = "Red_Package_1";
        else if (min_index_(vega_list) == 4)
            vega_package_name = "Yellow_Package_0";
        else if (min_index_(vega_list) == 5)
            vega_package_name = "Yellow_Package_1";

        vega_optimized.publish(vega_package_name);

        //icarus_Optimizer
        icarus_list.push_back(dist_bw_obj_mav_(blue_package_0_pose, icarus_odom_.pose.pose) + dist_bw_obj_mailbox_(blue_package_0_pose, blue_mailbox_pose));
        icarus_list.push_back(dist_bw_obj_mav_(blue_package_1_pose, icarus_odom_.pose.pose) + dist_bw_obj_mailbox_(blue_package_1_pose, blue_mailbox_pose));
        icarus_list.push_back(dist_bw_obj_mav_(red_package_0_pose, icarus_odom_.pose.pose) + dist_bw_obj_mailbox_(red_package_0_pose, red_mailbox_pose));
        icarus_list.push_back(dist_bw_obj_mav_(red_package_1_pose, icarus_odom_.pose.pose) + dist_bw_obj_mailbox_(red_package_1_pose, red_mailbox_pose));
        icarus_list.push_back(dist_bw_obj_mav_(yellow_package_0_pose, icarus_odom_.pose.pose) + dist_bw_obj_mailbox_(yellow_package_0_pose, yellow_mailbox_pose));
        icarus_list.push_back(dist_bw_obj_mav_(yellow_package_1_pose, icarus_odom_.pose.pose) + dist_bw_obj_mailbox_(yellow_package_1_pose, yellow_mailbox_pose));

        if (min_index_(icarus_list) == 0)
            icarus_package_name = "Blue_Package_0";
        else if (min_index_(icarus_list) == 1)
            icarus_package_name = "Blue_Package_1";
        else if (min_index_(icarus_list) == 2)
            icarus_package_name = "Red_Package_0";
        else if (min_index_(icarus_list) == 3)
            icarus_package_name = "Red_Package_1";
        else if (min_index_(icarus_list) == 4)
            icarus_package_name = "Yellow_Package_0";
        else if (min_index_(icarus_list) == 5)
            icarus_package_name = "Yellow_Package_1";

        icarus_optimized.publish(icarus_package_name);

        ros::spinOnce();
        loop_rate.sleep();
    }
}


// Points To be considered
// -> What if same distance
// -> Check for same dist for same quad 
// -> Currently only package is published but to consider for mailbox also and create a custom msg with package and mailbox string
// -> Catkin_simple 