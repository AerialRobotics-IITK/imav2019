#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


cv::Mat img,maskHSV,hsv, r_maskHSV, b_maskHSV, y_maskHSV, r_edges, y_edges, b_edges;
std::vector<std::vector<cv::Point> > r_contours, y_contours, b_contours;
std::vector<cv::Vec4i> r_hier, y_hier, b_hier;
double r_area, y_area, b_area, m_area;

int y_h_min, y_h_max, y_s_min , y_s_max , y_v_min, y_v_max ;
int r_h_min, r_h_max, r_s_min , r_s_max , r_v_min, r_v_max ;
int b_h_min, b_h_max, b_s_min , b_s_max , b_v_min, b_v_max ;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("colour_image", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("threshold_image", 1);

  }

  ~ImageConverter(){}

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    nh_.getParam("input/yellow/h_max", y_h_max);
    nh_.getParam("input/yellow/h_min", y_h_min);
    nh_.getParam("input/yellow/s_max", y_s_max);
    nh_.getParam("input/yellow/s_min", y_s_min);
    nh_.getParam("input/yellow/v_max", y_v_max);
    nh_.getParam("input/yellow/v_min", y_v_min);

    nh_.getParam("input/red/h_max", r_h_max);
    nh_.getParam("input/red/h_min", r_h_min);
    nh_.getParam("input/red/s_max", r_s_max);
    nh_.getParam("input/red/s_min", r_s_min);
    nh_.getParam("input/red/v_max", r_v_max);
    nh_.getParam("input/red/v_min", r_v_min);

    nh_.getParam("input/blue/h_max", b_h_max);
    nh_.getParam("input/blue/h_min", b_h_min);
    nh_.getParam("input/blue/s_max", b_s_max);
    nh_.getParam("input/blue/s_min", b_s_min);
    nh_.getParam("input/blue/v_max", b_v_max);
    nh_.getParam("input/blue/v_min", b_v_min);

    nh_.getParam("input/max_noise_area",m_area);

    r_area = 0; y_area = 0; b_area = 0;

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    img=cv_ptr->image;
    cv::cvtColor(img,hsv,CV_BGR2HSV);

    cv::inRange(hsv,cv::Scalar(r_h_min,r_s_min,r_v_min),cv::Scalar(r_h_max,r_s_max,r_v_max),r_maskHSV);
    cv::inRange(hsv,cv::Scalar(b_h_min,b_s_min,b_v_min),cv::Scalar(b_h_max,b_s_max,b_v_max),b_maskHSV);
    cv::inRange(hsv,cv::Scalar(y_h_min,y_s_min,y_v_min),cv::Scalar(y_h_max,y_s_max,y_v_max),y_maskHSV);

    cv::Canny(r_maskHSV, r_edges, 100, 100*2, 3);
    cv::Canny(b_maskHSV, b_edges, 100, 100*2, 3);
    cv::Canny(y_maskHSV, y_edges, 100, 100*2, 3);

    cv::findContours(r_edges, r_contours, r_hier, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0,0));    
    cv::findContours(b_edges, b_contours, b_hier, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0,0));
    cv::findContours(y_edges, y_contours, y_hier, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0,0));

    for(int i=0; i<r_contours.size(); i++)
    {
      if(cv::contourArea(r_contours[i])>m_area)
      {
        r_area += cv::contourArea(r_contours[i]);
      }
    }

    for(int i=0; i<y_contours.size(); i++)
    {
      if(cv::contourArea(y_contours[i])>m_area)
      {
        y_area += cv::contourArea(y_contours[i]);
      }
    }

    for(int i=0; i<b_contours.size(); i++)
    {
      if(cv::contourArea(b_contours[i])>m_area)
      {
        b_area += cv::contourArea(b_contours[i]);
      }
    }

    // std::cout << r_area << " " << y_area << " " << b_area << std::endl;
    
    if(r_area>b_area)
    {
      if(r_area>y_area)
      {
        maskHSV = r_maskHSV;
      }
      else
      {
        maskHSV = y_maskHSV;
      }
    }
    else
    {
      if(b_area>y_area)
      {
        maskHSV = b_maskHSV;
      }
      else
      {
        maskHSV = y_maskHSV;
      }
    }
    

    cv::cvtColor(maskHSV,img,cv::COLOR_GRAY2BGR);
    cv_bridge::CvImage in_msg;
    in_msg.header.stamp = ros::Time::now();
    in_msg.encoding = sensor_msgs::image_encodings::BGR8;
    in_msg.image = img;
    image_pub_.publish(in_msg.toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "input");
  ImageConverter ic;
  ros::spin();
  return 0;
}

