#include "aruco_ros/Load_Params.h"
#include "aruco_ros/Update.h"
#include "aruco_ros/Prediction.h"

int main(int argc, char** argv){

    ros::init(argc,argv,"KalmanFilter");
    ros::NodeHandle nh;     
  
    //Loading the parameters
    Load_Params(nh);
  
    //Subscriber and Publisher for the data. Can be remapped in the launch file to the topic required
    ros::Subscriber Image=nh.subscribe("Measurement_data",100,Update_step);
    ros::Subscriber Imu=nh.subscribe("Prediction_data",100,Prediction_step);
    ros::Publisher fused = nh.advertise<geometry_msgs::PoseStamped>("KF", 1000);

    ros::Rate loop_rate(100);
    while(ros::ok()){
        Load_Params(nh);
        if (/*fabs(output.pose.position.x)>0.1 && fabs(output.pose.position.y)>0.1 && fabs(output.pose.position.z) > 0.1*/1)fused.publish(output);
        //flag=0;
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}       
