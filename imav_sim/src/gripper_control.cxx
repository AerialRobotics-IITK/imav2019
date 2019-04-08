#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Imu.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/SetModelState.h>
#include <std_msgs/Bool.h>

using namespace std;

nav_msgs::Odometry mav_odom_;
geometry_msgs::Pose red_package_0_pose, red_package_1_pose,
                    yellow_package_0_pose, yellow_package_1_pose,
                    blue_package_0_pose, blue_package_1_pose;
std_msgs::Bool gripper_status;

void mav_cb(const nav_msgs::Odometry &msg)
{
    mav_odom_ = msg;
}

void gripper_cb(const std_msgs::Bool &msg)
{
    gripper_status = msg;
}

void obj_cb(const gazebo_msgs::ModelStates &msg)
{
    for (int i_=0; i_<(msg.name.size()); i_++)
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
    }   
}

float dist_from_est_pose_(geometry_msgs::Pose obj_pose, geometry_msgs::Pose est_pose_)
{
    return sqrt(pow((obj_pose.position.x - est_pose_.position.x), 2) + pow((obj_pose.position.y - est_pose_.position.y), 2)+ pow((obj_pose.position.z - est_pose_.position.z), 2));
}

int min_index_(vector<float> vec_)
{
    auto min_ = min_element(begin(vec_), end(vec_));
    return distance(begin(vec_), min_);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gripper_control");
    ros::NodeHandle nh;

    ros::Subscriber mav_sub_ = nh.subscribe("ground_truth/odometry", 100, mav_cb);
    ros::Subscriber obj_sub_ = nh.subscribe("/gazebo/model_states", 100, obj_cb);
    ros::Subscriber grip_status_sub_ = nh.subscribe("gripper_status", 100, gripper_cb);

    ros::ServiceClient set_pose_client_ = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

    gazebo_msgs::SetModelState set_obj_state;

    ros::Rate loop_rate(100);

    float last_position = 0;
    int i = 0, count = 0;
    float k_d = 200, k_p = 0.6;
    vector<float> dist_list;
    string package_name;
    gripper_status.data=false;
    while (ros::ok())
    {
        dist_list.push_back(dist_from_est_pose_(blue_package_0_pose, mav_odom_.pose.pose));
        dist_list.push_back(dist_from_est_pose_(blue_package_1_pose, mav_odom_.pose.pose));
        dist_list.push_back(dist_from_est_pose_(red_package_0_pose, mav_odom_.pose.pose));
        dist_list.push_back(dist_from_est_pose_(red_package_1_pose, mav_odom_.pose.pose));
        dist_list.push_back(dist_from_est_pose_(yellow_package_0_pose, mav_odom_.pose.pose));
        dist_list.push_back(dist_from_est_pose_(yellow_package_1_pose, mav_odom_.pose.pose));

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

        if (gripper_status.data && dist_list.at(min_index_(dist_list))<0.5)
        {
            set_obj_state.request.model_state.model_name = package_name;
            set_obj_state.request.model_state.pose.position = mav_odom_.pose.pose.position;
            set_obj_state.request.model_state.pose.position.z = mav_odom_.pose.pose.position.z - 0.20;
            set_obj_state.request.model_state.pose.orientation = mav_odom_.pose.pose.orientation;
            set_obj_state.request.model_state.twist = mav_odom_.twist.twist;

            if (set_pose_client_.call(set_obj_state))
            {
                ROS_INFO("Gripped %d", min_index_(dist_list));
            }
            else
            {
                ROS_ERROR("Failed to call service gripper");
                return 1;
            }
        }

        
        dist_list.clear();
        ros::spinOnce();
        loop_rate.sleep();
    }
}