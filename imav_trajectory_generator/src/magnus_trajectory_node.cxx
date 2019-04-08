#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/PoseStamped.h"
#include "ros/ros.h"
#include <mav_planning_msgs/PolynomialTrajectory4D.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation/timing.h>
#include <mav_trajectory_generation_ros/feasibility_analytic.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>

int main(int argc, char **argv)
{

    ros::init(argc, argv, "magnus_trajectory_node");
    ros::NodeHandle nh;

    ros::Publisher traj_pub_magnus = nh.advertise<visualization_msgs::MarkerArray>("trajectory", 10);
    
    ros::Publisher polynomial_trajectory_pub_magnus = nh.advertise<mav_planning_msgs::PolynomialTrajectory4D>("path_segments", 1);

    ros::Rate sleep_Rate(1);

    while (ros::ok())
    {

        mav_trajectory_generation::Vertex::Vector magnus_vertices;

        const int dimension = 3;
        const int derivative_to_optimize = mav_trajectory_generation::derivative_order::VELOCITY;
        mav_trajectory_generation::Vertex start(dimension), middle(dimension), end(dimension);

        // Eigen::Vector3d magnus_start;
        // magnus_start(0,5,6);

        //Trajectory points :: Magnus

        start.makeStartOrEnd(Eigen::Vector3d(0, 5, 6), derivative_to_optimize);
        magnus_vertices.push_back(start);

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(325, 5, 6));
        magnus_vertices.push_back(middle);

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(325, 5 + 10, 6));
        magnus_vertices.push_back(middle);

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(5, 5 + 10, 6));
        magnus_vertices.push_back(middle);

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(5, 5 + 20, 6));
        magnus_vertices.push_back(middle);

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(325, 5 + 20, 6));
        magnus_vertices.push_back(middle);

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(325, 5 + 30, 6));
        magnus_vertices.push_back(middle);

        end.makeStartOrEnd(Eigen::Vector3d(5, 5 + 30, 6), derivative_to_optimize);
        magnus_vertices.push_back(end);


        std::vector<double> magnus_segment_times;
        
        const double v_max = 5;
        const double a_max = 5;
        const double magic_fabian_constant = 6.5; // A tuning parameter.
        magnus_segment_times = estimateSegmentTimes(magnus_vertices, v_max, a_max);
    
        const int N = 10;
        mav_trajectory_generation::PolynomialOptimization<N> magnus_opt(dimension);
       
        magnus_opt.setupFromVertices(magnus_vertices, magnus_segment_times, derivative_to_optimize);
        magnus_opt.solveLinear();

        mav_trajectory_generation::Segment::Vector magnus_segments;
        magnus_opt.getSegments(&magnus_segments);
        mav_trajectory_generation::Trajectory magnus_trajectory;
        magnus_opt.getTrajectory(&magnus_trajectory);


        double sampling_time = 2.0;
        int derivative_order = mav_trajectory_generation::derivative_order::POSITION;
        Eigen::VectorXd magnus_sample = magnus_trajectory.evaluate(sampling_time, derivative_order);
    
        double t_start = 2.0;
        double t_end = 10.0;
        double dt = 0.01;
        std::vector<Eigen::VectorXd> magnus_result;
        std::vector<double> sampling_times; // Optional.
        magnus_trajectory.evaluateRange(t_start, t_end, dt, derivative_order, &magnus_result, &sampling_times);

        visualization_msgs::MarkerArray magnus_markers;
        double distance = 1.0; // Distance by which to seperate additional markers. Set 0.0 to disable.
        std::string frame_id = "world";

        mav_trajectory_generation::drawMavTrajectory(magnus_trajectory, distance, frame_id, &magnus_markers);
        mav_planning_msgs::PolynomialTrajectory4D magnus_msg;
        mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(magnus_trajectory, &magnus_msg);
        magnus_msg.header.stamp = ros::Time::now();
        magnus_msg.header.frame_id = "world";
        polynomial_trajectory_pub_magnus.publish(magnus_msg);

        ros::spinOnce();

        traj_pub_magnus.publish(magnus_markers);

        magnus_vertices.clear();

        sleep_Rate.sleep();
    }
    return 0;
}