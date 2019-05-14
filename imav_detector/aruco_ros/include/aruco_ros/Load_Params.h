#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <bits/stdc++.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <math.h>
#include <ros/ros.h>
#include <stdio.h>

using namespace Eigen;

/*
Standard matrices for the Kalman Filter
Fk - State-Transition model
Qk - Process-Noise Covariance
Rk - Observation-Noise Covariance
Bk - Control-Input Model
Hk - Observation-Model
*/
MatrixXf Fk;
MatrixXf Qk;
MatrixXf Rk;
MatrixXf Bk;
MatrixXf Hk;

/*
Observation and Measurement Step Vectors/Matrices
Xk - Current state vector(Belief)
Xhatk - Predicted state vector
Zk - Measurement vector
Uk - Control Vector
covarx - Current covariance of X
covarhatx - Predicted covariance of X
KGain - Kalman Gain
*/
MatrixXf Xk,Xhatk; 
MatrixXf Zk,Uk;
MatrixXf CovarX,CovarhatX;
MatrixXf KGain;

/*
Miscellaneous vectors/matrices for converting incoming data into a useable form.
Can be omitted/changed according to needs  
*/
MatrixXf g(3,1);
MatrixXf imutemp(3,1);  
MatrixXf rotmat(3,3);
Quaternionf q;

//Output to be published. Can be of different type for different needs
geometry_msgs::PoseStamped output;



void Load_Params(ros::NodeHandle nh){
	
	//Information about rows and columns of the required matrices
	int Hkrows, Hkcolumns;
	int Qkrows, Qkcolumns;
	int Fkrows, Fkcolumns;
	int Bkrows, Bkcolumns;
	int Rkrows, Rkcolumns;
	int Covar_order;
	int Xkorder, Zkorder, Ukorder;
	int KGainr, KGainc;

	//Loading the above information from parameters
	nh.getParam("/icarus/KalmanFilter/Hkr",Hkrows);
	nh.getParam("/icarus/KalmanFilter/Hkc",Hkcolumns);
	nh.getParam("/icarus/KalmanFilter/Qkr",Qkrows);
	nh.getParam("/icarus/KalmanFilter/Qkc",Qkcolumns);
	nh.getParam("/icarus/KalmanFilter/Fkr",Fkrows);
	nh.getParam("/icarus/KalmanFilter/Fkc",Fkcolumns);
	nh.getParam("/icarus/KalmanFilter/Bkr",Bkrows);
	nh.getParam("/icarus/KalmanFilter/Bkc",Bkcolumns);
	nh.getParam("/icarus/KalmanFilter/Rkr",Rkrows);
	nh.getParam("/icarus/KalmanFilter/Rkc",Rkcolumns);
	nh.getParam("/icarus/KalmanFilter/Cov_order",Covar_order);
	nh.getParam("/icarus/KalmanFilter/Xk",Xkorder);
	nh.getParam("/icarus/KalmanFilter/Zk",Zkorder);
	nh.getParam("/icarus/KalmanFilter/Uk",Ukorder);
	nh.getParam("/icarus/KalmanFilter/Kgainr",KGainr);
	nh.getParam("/icarus/KalmanFilter/Kgainc",KGainc);

	//Making the matrices of the given size
	Hk.resize(Hkrows,Hkcolumns);
	Qk.resize(Qkrows, Qkcolumns);
	Fk.resize(Fkrows,Fkcolumns);
	Bk.resize(Bkrows,Bkcolumns);
	Rk.resize(Rkrows,Rkcolumns);
	CovarX.resize(Covar_order,Covar_order);
	CovarhatX.resize(Covar_order,Covar_order);
	Xk.resize(Xkorder,1);
	Xhatk.resize(Xkorder,1);
	Zk.resize(Zkorder,1);
	Uk.resize(Ukorder,1);
	KGain.resize(KGainr,KGainc);


	float temp;

	//Loading the given Matrices from the Parameters
	char HkStr[]="/icarus/KalmanFilter/Hk00";
	for (int i=0;i<Hkrows;i++){
		for (int j=0;j<Hkcolumns;j++){
			HkStr[23]='0'+i;
			HkStr[24]='0'+j;
			nh.getParam(HkStr,temp);
			Hk(i,j)=temp;
		}
	}

	char QkStr[]="/icarus/KalmanFilter/Qk00";	
	for (int i=0;i<Qkrows;i++){
		for (int j=0;j<Qkcolumns;j++){
			QkStr[23]='0'+i;
			QkStr[24]='0'+j;
			nh.getParam(QkStr,temp);
			Qk(i,j)=temp;
		}
	}

	char FkStr[]="/icarus/KalmanFilter/Fk00";	
	for (int i=0;i<Fkrows;i++){
		for (int j=0;j<Fkcolumns;j++){
			FkStr[23]='0'+i;
			FkStr[24]='0'+j;
			nh.getParam(FkStr,temp);
			Fk(i,j)=temp;
		}
	}

	char BkStr[]="/icarus/KalmanFilter/Bk00";	
	for (int i=0;i<Bkrows;i++){
		for (int j=0;j<Bkcolumns;j++){
			BkStr[23]='0'+i;
			BkStr[24]='0'+j;
			nh.getParam(BkStr,temp);
			Bk(i,j)=temp;
		}
	}

	char RkStr[]="/icarus/KalmanFilter/Rk00";	
	for (int i=0;i<Rkrows;i++){
		for (int j=0;j<Rkcolumns;j++){
			RkStr[23]='0'+i;
			RkStr[24]='0'+j;
			nh.getParam(RkStr,temp);
			Rk(i,j)=temp;
		}
	}   

	for (int i=0;i<Covar_order;i++){
		for (int j=0;j<Covar_order;j++){
			CovarX(i,j)=(i==j)?1:0;
		}
	}

	std::cout << Hk << std::endl << std::endl;
	std::cout << Qk << std::endl << std::endl;
	std::cout << Fk << std::endl << std::endl;
	std::cout << Fk*Fk.transpose() << std::endl << std::endl;
	std::cout << Bk << std::endl << std::endl;
	std::cout << Rk << std::endl << std::endl;
	std::cout << CovarX;
}
