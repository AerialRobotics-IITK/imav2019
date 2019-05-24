#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <aruco_ros/object.h>

#define WHITE (255,255,255)
#define BLACK (0,0,0)


cv::Mat img, maskHSV, hsv;
cv::Mat r_maskHSV, b_maskHSV, y_maskHSV;
cv::Mat blurred,opened,morphed;
double var=0.6;

// cv::Mat r_edges, y_edges, b_edges;
// cv::Mat y_blurred, y_opened, y_morphed;
// cv::Mat r_blurred, r_opened, r_morphed;
// cv::Mat b_blurred, b_opened, b_morphed;
unsigned long int r_area, y_area, b_area;

int y_h_min, y_h_max, y_s_min , y_s_max , y_v_min, y_v_max ;
int r_h_min, r_h_max, r_s_min , r_s_max , r_v_min, r_v_max ;
int b_h_min, b_h_max, b_s_min , b_s_max , b_v_min, b_v_max ;
int ksize=9, open_ksize=3, close_ksize=9, sigma=75;


cv::Mat image_threshold(cv::Mat img ,double* marker_size,ros::NodeHandle nh_)
{
static int count=0;
if (count==0)
  { nh_.getParam("/aruco_single/yellow/h_max", y_h_max);
    nh_.getParam("/aruco_single/yellow/h_min", y_h_min);
    nh_.getParam("/aruco_single/yellow/s_max", y_s_max);
    nh_.getParam("/aruco_single/yellow/s_min", y_s_min);
    nh_.getParam("/aruco_single/yellow/v_max", y_v_max);
    nh_.getParam("/aruco_single/yellow/v_min", y_v_min);

    nh_.getParam("/aruco_single/red/h_max", r_h_max);
    nh_.getParam("/aruco_single/red/h_min", r_h_min);
    nh_.getParam("/aruco_single/red/s_max", r_s_max);
    nh_.getParam("/aruco_single/red/s_min", r_s_min);
    nh_.getParam("/aruco_single/red/v_max", r_v_max);
    nh_.getParam("/aruco_single/red/v_min", r_v_min);
    //ROS_INFO_STREAM("hue max: " << r_h_max << "  min: " << r_h_min);

    nh_.getParam("/aruco_single/blue/h_max", b_h_max);
    nh_.getParam("/aruco_single/blue/h_min", b_h_min);
    nh_.getParam("/aruco_single/blue/s_max", b_s_max);
    nh_.getParam("/aruco_single/blue/s_min", b_s_min);
    nh_.getParam("/aruco_single/blue/v_max", b_v_max);
    nh_.getParam("/aruco_single/blue/v_min", b_v_min);

    nh_.getParam("/aruco_single/bilateral/ksize", ksize);
    nh_.getParam("/aruco_single/bilateral/sigma", sigma);
    nh_.getParam("/aruco_single/morph_open/ksize", open_ksize);
    nh_.getParam("/aruco_single/morph_close/ksize", close_ksize);
}
    r_area = 0; y_area = 0; b_area = 0;

    cv::cvtColor(img,hsv,CV_BGR2HSV);

    cv::inRange(hsv,cv::Scalar(r_h_min,r_s_min,r_v_min),cv::Scalar(r_h_max,r_s_max,r_v_max),r_maskHSV);
    cv::inRange(hsv,cv::Scalar(b_h_min,b_s_min,b_v_min),cv::Scalar(b_h_max,b_s_max,b_v_max),b_maskHSV);
    cv::inRange(hsv,cv::Scalar(y_h_min,y_s_min,y_v_min),cv::Scalar(y_h_max,y_s_max,y_v_max),y_maskHSV);

    r_area=cv::countNonZero(r_maskHSV);
    b_area=cv::countNonZero(b_maskHSV);
    y_area=cv::countNonZero(y_maskHSV);

    // for(int i=0; i<img.rows; i++)
    // {
    //   for(int j=0; j<img.cols;j++)
    //   {
    //     if(y_maskHSV.at<uchar>(i,j) != 0)
    //       y_area++;
    //     if(b_maskHSV.at<uchar>(i,j) != 0)
    //       b_area++;  
    //     if(r_maskHSV.at<uchar>(i,j) != 0)
    //      r_area++;

    //   }
    // }

    if(r_area>b_area)
    {
      if(r_area>y_area)
      {
        maskHSV = r_maskHSV;
        
        *marker_size=var;
      }
      else
      {
        maskHSV = y_maskHSV;
        *marker_size=var;
      }
    }
    else
    {
      if(b_area>y_area)
      {
        maskHSV = b_maskHSV;
        *marker_size=var;
      }
      else
      {
        maskHSV = y_maskHSV;
        *marker_size=var;
      }
    }
    

    // cv::bilateralFilter(maskHSV, blurred,ksize,sigma,sigma);
    // cv::morphologyEx(blurred, opened, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(open_ksize,open_ksize), cv::Point(1,1)));
    // cv::morphologyEx(opened, morphed, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(close_ksize,close_ksize), cv::Point(1,1)));

   // cv::bilateralFilter(maskHSV, blurred,ksize,sigma,sigma);
    cv::morphologyEx(maskHSV, opened, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(open_ksize,open_ksize), cv::Point(1,1)));
    cv::morphologyEx(opened, morphed, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(close_ksize,close_ksize), cv::Point(1,1)));

    cv::cvtColor(morphed,img,cv::COLOR_GRAY2BGR);
    //cv::imshow("image",img);
    return img;
    // cv::GaussianBlur(y_maskHSV, y_blurred, cv::Size(5,5), 0, 0);
    // cv::GaussianBlur(r_maskHSV, r_blurred, cv::Size(5,5), 0, 0);
    // cv::GaussianBlur(b_maskHSV, b_blurred, cv::Size(5,5), 0, 0);
    
    // cv::bilateralFilter(y_maskHSV, y_blurred,ksize,sigma,sigma);
    // cv::bilateralFilter(r_maskHSV, r_blurred,ksize,sigma,sigma);
    // cv::bilateralFilter(b_maskHSV, b_blurred,ksize,sigma,sigma);

    // cv::medianBlur( y_maskHSV, y_blurred,5 );
    // cv::medianBlur( r_maskHSV, r_blurred,5  );
    // cv::medianBlur( b_maskHSV, b_blurred,5 );

    // cv::morphologyEx(y_blurred, y_opened, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(open_ksize,open_ksize), cv::Point(1,1)));
    // cv::morphologyEx(r_blurred, r_opened, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(open_ksize,open_ksize), cv::Point(1,1)));
    // cv::morphologyEx(b_blurred, b_opened, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(open_ksize,open_ksize), cv::Point(1,1)));
    
    // cv::morphologyEx(y_opened, y_morphed, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(close_ksize,close_ksize), cv::Point(1,1)));
    // cv::morphologyEx(r_opened, r_morphed, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(close_ksize,close_ksize), cv::Point(1,1)));
    // cv::morphologyEx(b_opened, b_morphed, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(close_ksize,close_ksize), cv::Point(1,1)));

    //std::cout << "pixel" << y_morphed.at<uchar>(0, 0) << std::endl;
    count=1;
}


