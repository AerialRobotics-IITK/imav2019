#include<ros/ros.h>
#include<sensor_msgs/Image.h>

sensor_msgs::Image magnus_image;
sensor_msgs::Image vega_image;
sensor_msgs::Image icarus_image;

void Magnus_Image_Callback(const sensor_msgs::Image& msg)
{
    magnus_image=msg;
}

void Vega_Image_Callback(const sensor_msgs::Image& msg)
{
    vega_image=msg;
}

void Icarus_Image_Callback(const sensor_msgs::Image& msg)
{
    icarus_image=msg;
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"detector");
    ros::NodeHandle nh;

    ros::Subscriber magnus_subscriber = nh.subscribe("magnus/camera",10,Magnus_Image_Callback);
    ros::Subscriber vega_subscriber = nh.subscribe("vega/camera",10,Vega_Image_Callback);
    ros::Subscriber icarus_subscriber = nh.subscribe("icarus/camera",10,Icarus_Image_Callback);

    ros::Rate loopRate(10);

    while(ros::ok())
    {
        ros::spinOnce();
        loopRate.sleep();
    }

    return 0;
}