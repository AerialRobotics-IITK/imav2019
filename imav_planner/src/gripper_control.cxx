#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Imu.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/SetModelState.h>

using namespace std;

geometry_msgs::Pose current_pose;
double vel_z = 0, roll_ = 0, pitch_ = 0;

geometry_msgs::Pose mav1_pose, mav2_pose, mav3_pose, 
                    red_package_0_pose, red_package_1_pose,
                    yellow_package_0_pose, yellow_package_1_pose,
                    blue_package_0_pose, blue_package_1_pose;

gazebo_msgs::SetModelState set_obj_state;

void mav1_cb(const geometry_msgs::Pose &msg)
    mav1_pose = msg;

void mav2_cb(const geometry_msgs::Pose &msg)
    mav2_pose = msg;

void mav2_cb(const geometry_msgs::Pose &msg)
    mav3_pose = msg;

void obj_cb(const gazebo_msgs::ModelStates &msg)
{
    for (int i_=0; i_<size(msg.string); i_++)
    {
        string obj_name = msg.string[i_];
        if (obj_name == "Blue_Package")
            blue_package_0_pose = msg.pose[i_];
        if (obj_name == "Blue_Package_0")
            blue_package_1_pose = msg.pose[i_];
        if (obj_name == "Yellow_Package")
            yellow_package_0_pose = msg.pose[i_];
        if (obj_name == "Yellow_Package_0")
            yellow_package_1_pose = msg.pose[i_];
        if (obj_name == "Red_Package_0")
            red_package_0_pose = msg.pose[i_];
        if (obj_name == "red_Package_1")
            red_package_1_pose = msg.pose[i_];
    }   
}

void obj_state_set(string package_name)
{
    set_obj_state.
}

void imu_sub(const sensor_msgs::Imu &msg)
{

    tf::Quaternion quat_(msg.orientation.x,
                         msg.orientation.y,
                         msg.orientation.z,
                         msg.orientation.w);
    tf::Matrix3x3 m1(quat_);
    double yaw_marker = 0;
    m1.getRPY(roll_, pitch_, yaw_marker);
    cout << "roll" << roll_ << endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gripper_control");
    ros::NodeHandle nh;
    ros::Publisher mav1_sub =nh.advertise<geometry_msgs::PoseStamped>("/firefly/command/pose",10);
    ros::Publisher thrust_pub = nh.advertise<mav_msgs::RollPitchYawrateThrust>("/mavros/setpoint_raw/roll_pitch_yawrate_thrust", 10);
    ros::Subscriber current_pose_sub = nh.subscribe("/mavros/local_position/odom", 100, pose_sub);
    ros::Subscriber mav1_sub_ = nh.subscribe("/magnus/ground_truth/pose", 100, mav1_cb);
    ros::Subscriber mav2_sub_ = nh.subscribe("/vega/ground_truth/pose", 100, mav2_cb);
    ros::Subscriber mav3_sub_ = nh.subscribe("/icarus/ground_truth/pose", 100, mav3_cb);
    ros::Subscriber obj_sub_ = nh.subscribe("/gazebo/model_states ", 100, obj_cb);
    ros::ServiceClient set_pose_client_ = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    ros::Rate loop_rate(100);

    float last_position = 0;
    int i = 0, count = 0;
    float k_d = 200, k_p = 0.6;

    while (ros::ok())
    {
        string mode_;
        nh.getParam("/flip_node/mode_", mode_);
        nh.getParam("/flip_node/k_p", k_p);
        nh.getParam("/flip_node/k_d", k_d);

        geometry_msgs::Pose msg;
        msg.position.z = current_pose.position.z;

        float error_z;
        mav_msgs::RollPitchYawrateThrust thrust_cmd;
        thrust_cmd.header.stamp = ros::Time::now();
        error_z = 3 - msg.position.z;
        if (mode_ == "hover")
        {
            if (i > 0)
                thrust_cmd.thrust.z = (19.1 + (error_z * k_p) - ((msg.position.z - last_position) * k_d));
            // thrust_cmd.roll = -0.1;
            // cout<< "D term"<< -(msg.position.z-last_position)*k_d<< endl;
        }
        else if (mode_ == "flip")
        {
            if ((vel_z < 3) && (roll_ < 1 && roll_ > -1) && count < 1)
                thrust_cmd.thrust.z = 100;
            else
            {
                if (count < 30)
                {
                    thrust_cmd.roll = 3.1;
                    thrust_cmd.thrust.z = 15;
                    count = count + 1;
                }
                else
                    thrust_cmd.thrust.z = 0;

                if ((roll_ < -0.2) && (roll_ > -1))
                {
                    nh.setParam("/flip_node/mode_", "hover");
                    count = 0;
                    // cout << "roll" << roll_ << endl;
                }
                // cout << "roll" << roll_ << endl;
            }
        }

        thrust_pub.publish(thrust_cmd);

        last_position = msg.position.z;
        i = i + 1;
        // std::cout<<"The height is "<< msg.position.z << std::endl;
        ros::spinOnce();
        loop_rate.sleep();
    }
}