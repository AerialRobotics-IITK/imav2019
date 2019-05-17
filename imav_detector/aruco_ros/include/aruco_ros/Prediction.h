/*
Call Back function for obtaining the control vector
The argument can be changed to that of incoming message
*/

void Prediction_step(const sensor_msgs::Imu::ConstPtr& control){
    
    //Preprocessing for the control vector. Can be changed/omitted
    float Ax,Ay,Az;
    float Angx,Angy,Angz;    
    Quaternionf q,qrev;
    q = Quaternionf(control->orientation.w, control->orientation.x, control->orientation.y, control->orientation.z);
    MatrixXf rotmat = q.toRotationMatrix();
    MatrixXf rotmat2=rotmat.inverse();

    Ax=control->linear_acceleration.x;
    Ay=control->linear_acceleration.y;
    Az=control->linear_acceleration.z;
    imutemp << Ax,Ay,Az;
    g << 0,0,-9.8;
    Uk =imutemp+rotmat2*g; 
    

    //Predicting belief of x
    Xhatk=Fk*Xk+Bk*Uk;

    //Predicting covariance of x
    CovarhatX=(Fk*CovarX)*Fk.transpose()+Qk;

    //Calculating kalman gain
    MatrixXf temp2,temp3;
    temp2=(Hk*CovarhatX)*Hk.transpose()+Rk;
    temp3=temp2.inverse();
    KGain=(CovarhatX*Hk.transpose())*temp3;
    //std::cout << Xk << std::endl << std::endl; 

    output.pose.position.x=Xk(0,0);
    output.pose.position.y=Xk(1,0);
    output.pose.position.z=Xk(2,0); 
}