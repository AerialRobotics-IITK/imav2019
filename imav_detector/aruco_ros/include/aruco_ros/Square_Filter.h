cv::Mat sqr_filter (cv::Mat filtered, ros::NodeHandle _nh) {

    int sqr_size,wb_ratio;
    _nh.getParam("input/sqr_size",sqr_size);
    _nh.getParam("input/wb_ratio",wb_ratio);  
    for (int i=0;i<filtered.rows/40;i++){
      for (int j=0;j<filtered.rows/40;j++){
       int whitepxs=0, blackpxs=0;
        
        for (int k=0;k<40;k++) for (int l=0;l<40;l++) if (filtered.at<uchar>(i*40+k,j*40+l)==0) blackpxs++;
        whitepxs=1600-blackpxs;

        float ratio=((float)whitepxs)/blackpxs;
        if (ratio <0.25) for (int k=0;k<40;k++) for (int l=0;l<40;l++) filtered.at<uchar>(i*40+k,j*40+l)=0;
      }
    }

    return filtered;
}