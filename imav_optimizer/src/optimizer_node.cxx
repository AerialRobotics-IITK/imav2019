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

nav_msgs::Odometry mav_odom_;

geometry_msgs::Pose red_package_0_pose, red_package_1_pose, red_mailbox_pose,
    yellow_package_0_pose, yellow_package_1_pose, yellow_mailbox_pose,
    blue_package_0_pose, blue_package_1_pose ,blue_mailbox_pose;

void mav_cb(const nav_msgs::Odometry &msg)
{
    mav_odom_ = msg;
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

    ros::Subscriber mav_sub_ = nh.subscribe("ground_truth/odometry", 100, mav_cb);
    ros::Subscriber obj_sub_ = nh.subscribe("/gazebo/model_states", 100, obj_cb);
    ros::Publisher mav_optimized = nh.advertise<std_msgs::String>("optimized",1);

    ros::Rate loop_rate(100);

    vector<float> dist_list;
    string package_name;

    while (ros::ok())
    {   
        //Current Idea: To pick object having min(dist_from_quad+dist_from_obj_to_mailbox)
    
        dist_list.push_back(dist_bw_obj_mav_(blue_package_0_pose, mav_odom_.pose.pose) + dist_bw_obj_mailbox_(blue_package_0_pose, blue_mailbox_pose));
        dist_list.push_back(dist_bw_obj_mav_(blue_package_1_pose, mav_odom_.pose.pose) + dist_bw_obj_mailbox_(blue_package_1_pose, blue_mailbox_pose));
        dist_list.push_back(dist_bw_obj_mav_(red_package_0_pose, mav_odom_.pose.pose) + dist_bw_obj_mailbox_(red_package_0_pose, red_mailbox_pose));
        dist_list.push_back(dist_bw_obj_mav_(red_package_1_pose, mav_odom_.pose.pose) + dist_bw_obj_mailbox_(red_package_1_pose, red_mailbox_pose));
        dist_list.push_back(dist_bw_obj_mav_(yellow_package_0_pose, mav_odom_.pose.pose) + dist_bw_obj_mailbox_(yellow_package_0_pose, yellow_mailbox_pose));
        dist_list.push_back(dist_bw_obj_mav_(yellow_package_1_pose, mav_odom_.pose.pose) + dist_bw_obj_mailbox_(yellow_package_1_pose, yellow_mailbox_pose));

        if (min_index_(dist_list) == 0)
            package_name = "Blue_Package_0";
        else if (min_index_(dist_list) == 1)
            package_name = "Blue_Package_1";
        else if (min_index_(dist_list) == 2)
            package_name = "Red_Package_0";
        else if (min_index_(dist_list) == 3)
            package_name = "Red_Package_1";
        else if (min_index_(dist_list) == 4)
            package_name = "Yellow_Package_0";
        else if (min_index_(dist_list) == 5)
            package_name = "Yellow_Package_1";

        mav_optimized.publish(package_name);

        dist_list.clear();
        ros::spinOnce();
        loop_rate.sleep();
    }
}


// Points To be considered
// -> What if same distance
// -> Check for same dist for same quad 
// -> Currently only package is published but to consider for mailbox also and create a custom msg with package and mailbox string
// -> Catkin_simple 