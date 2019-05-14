/*
Call Back function for obtaining the measurement vector.
The argument can be changed to that of incoming message
*/

void Update_step(const geometry_msgs::PoseStamped::ConstPtr& measurement){

    //Preproccessing for making the output message ready for publishing. Can be changed/omitted
    float xx,yy,zz;
    xx=measurement->pose.position.x;
    yy=measurement->pose.position.y;
    zz=measurement->pose.position.z;
    Zk << xx,yy,zz;

    output.header.stamp=measurement->header.stamp;
    output.header.seq=measurement->header.seq;
    output.header.frame_id=measurement->header.frame_id;
    output.pose.orientation.x=measurement->pose.orientation.x;
    output.pose.orientation.y=measurement->pose.orientation.y;
    output.pose.orientation.z=measurement->pose.orientation.z;
    output.pose.orientation.w=measurement->pose.orientation.w;
    
    //For the first value, set initial belief. Can be set to whatever you fancy
    static int ifFirst=1;
    if (ifFirst){
        ifFirst=0;
        Xk <<   measurement->pose.position.x,
                measurement->pose.position.y,
                measurement->pose.position.z,
                0,
                0,
                0;
        return;
    }
    //Calculating new belief of x
    Xk=Xhatk+KGain*(Zk-(Hk*Xhatk));

    //Calculating new covariance of x
    CovarX=CovarhatX-((KGain*Hk)*CovarhatX);
}