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

int count=0;

int main(int argc, char **argv)
{

    ros::init(argc, argv, "vega_trajectory_node");
    ros::NodeHandle nh;

    ros::Publisher traj_pub_vega = nh.advertise<visualization_msgs::MarkerArray>("trajectory", 10);

    ros::Publisher polynomial_trajectory_pub_vega = nh.advertise<mav_planning_msgs::PolynomialTrajectory4D>("path_segments", 1);

    ros::Rate sleep_Rate(1);

    while (ros::ok())
    {

        mav_trajectory_generation::Vertex::Vector vega_vertices;

        const int dimension = 3;
        const int derivative_to_optimize = mav_trajectory_generation::derivative_order::VELOCITY;
        mav_trajectory_generation::Vertex start(dimension), middle(dimension), end(dimension);

        // Eigen::Vector3d vega_start;
        // vega_start(0,5,6);

        //Trajectory points :: Vega

        start.makeStartOrEnd(Eigen::Vector3d(0, 42, 6), derivative_to_optimize);
        vega_vertices.push_back(start);

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(325, 42, 6));
        vega_vertices.push_back(middle);

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(325, 42 + 10, 6));
        vega_vertices.push_back(middle);

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(5, 42 + 10, 6));
        vega_vertices.push_back(middle);

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(5, 42 + 20, 6));
        vega_vertices.push_back(middle);

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(325, 42 + 20, 6));
        vega_vertices.push_back(middle);

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(325, 42 + 30, 6));
        vega_vertices.push_back(middle);

        end.makeStartOrEnd(Eigen::Vector3d(5, 42 + 30, 6), derivative_to_optimize);
        vega_vertices.push_back(end);
        
        std::vector<double> vega_segment_times;

        const double v_max = 10;
        const double a_max = 5;
        const double magic_fabian_constant = 6.5; // A tuning parameter.
        vega_segment_times = estimateSegmentTimes(vega_vertices, v_max, a_max);

        const int N = 10;
        mav_trajectory_generation::PolynomialOptimization<N> vega_opt(dimension);

        vega_opt.setupFromVertices(vega_vertices, vega_segment_times, derivative_to_optimize);
        vega_opt.solveLinear();

        mav_trajectory_generation::Segment::Vector vega_segments;
        vega_opt.getSegments(&vega_segments);
        mav_trajectory_generation::Trajectory vega_trajectory;
        vega_opt.getTrajectory(&vega_trajectory);

        double sampling_time = 2.0;
        int derivative_order = mav_trajectory_generation::derivative_order::POSITION;
        Eigen::VectorXd vega_sample = vega_trajectory.evaluate(sampling_time, derivative_order);

        double t_start = 2.0;
        double t_end = 10.0;
        double dt = 0.01;
        std::vector<Eigen::VectorXd> vega_result;
        std::vector<double> sampling_times; // Optional.
        vega_trajectory.evaluateRange(t_start, t_end, dt, derivative_order, &vega_result, &sampling_times);

        visualization_msgs::MarkerArray vega_markers;
        double distance = 1.0; // Distance by which to seperate additional markers. Set 0.0 to disable.
        std::string frame_id = "world";

        mav_trajectory_generation::drawMavTrajectory(vega_trajectory, distance, frame_id, &vega_markers);
        mav_planning_msgs::PolynomialTrajectory4D vega_msg;
        mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(vega_trajectory, &vega_msg);
        vega_msg.header.stamp = ros::Time::now();
        vega_msg.header.frame_id = "world";
        if(count<10)
        {
        polynomial_trajectory_pub_vega.publish(vega_msg);
        }

        ros::spinOnce();

        traj_pub_vega.publish(vega_markers);

        vega_vertices.clear();

        count++;
        sleep_Rate.sleep();
    }
    return 0;
}