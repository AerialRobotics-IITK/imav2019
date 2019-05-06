#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <aruco_ros/drop_info.h>

nav_msgs::Odometry odom_msg_;
geometry_msgs::PoseStamped obj_msg_;

void odom_cb_(const nav_msgs::Odometry &msg)
{
    odom_msg_ = msg;
}

void obj_cb_(const geometry_msgs::PoseStamped &msg)
{
    obj_msg_ = msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "global_pos");
    ros::NodeHandle nh;

    ros::Subscriber odom_sub_ = nh.subscribe("tfmini_odom", 100, odom_cb_);
    ros::Subscriber obj_sub_ = nh.subscribe("filter/pose", 100, obj_cb_);
    ros::Publisher msg_pub_ = nh.advertise<aruco_ros::drop_info>("drop_info", 100);

    aruco_ros::drop_info msg_;

    while(ros::ok())
    {
        ros::spinOnce();

        msg_.header = odom_msg_.header;
        msg_.loc_type = "Drop";

        // add object colour here
        msg_.mailbox_color = "";

        msg_.position.x = odom_msg_.pose.pose.position.x + obj_msg_.pose.position.x;
        msg_.position.y = odom_msg_.pose.pose.position.y + obj_msg_.pose.position.y;
        
        // parametrize this mission height
        msg_.position.z = 4.0;

        msg_pub_.publish(msg_);    
    }

    return 0;
}
