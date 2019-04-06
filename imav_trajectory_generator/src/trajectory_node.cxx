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

    ros::init(argc, argv, "trajectory_node");
    ros::NodeHandle nh;

    ros::Publisher traj_pub_magnus = nh.advertise<visualization_msgs::MarkerArray>("magnus_trajectory", 10);
    ros::Publisher traj_pub_vega = nh.advertise<visualization_msgs::MarkerArray>("vega_trajectory", 10);
    ros::Publisher traj_pub_icarus = nh.advertise<visualization_msgs::MarkerArray>("icarus_trajectory", 10);

    ros::Publisher polynomial_trajectory_pub_magnus = nh.advertise<mav_planning_msgs::PolynomialTrajectory4D>("magnus_path_segments", 1);
    ros::Publisher polynomial_trajectory_pub_vega = nh.advertise<mav_planning_msgs::PolynomialTrajectory4D>("vega_path_segments", 1);
    ros::Publisher polynomial_trajectory_pub_icarus = nh.advertise<mav_planning_msgs::PolynomialTrajectory4D>("icarus_path_segments", 1);

    ros::Rate sleep_Rate(1);

    while (ros::ok())
    {

        mav_trajectory_generation::Vertex::Vector magnus_vertices;
        mav_trajectory_generation::Vertex::Vector vega_vertices;
        mav_trajectory_generation::Vertex::Vector icarus_vertices;

        const int dimension = 3;
        const int derivative_to_optimize = mav_trajectory_generation::derivative_order::VELOCITY;
        mav_trajectory_generation::Vertex start(dimension), middle(dimension), end(dimension);

        // Eigen::Vector3d magnus_start;
        // Eigen::Vector3d vega_start;
        // Eigen::Vector3d icarus_start;
        // magnus_start(0,5,6);
        // vega_start(0,42,6);
        // icarus_start(0,79,6);

        //Trajectory points :: Magnus

        start.makeStartOrEnd(Eigen::Vector3d(0,5,6), derivative_to_optimize);
        magnus_vertices.push_back(start);

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(325,5,6));
        magnus_vertices.push_back(middle);

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(325, 5+10, 6));
        magnus_vertices.push_back(middle);

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(5, 5+10, 6));
        magnus_vertices.push_back(middle);

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(5, 5 + 20, 6));
        magnus_vertices.push_back(middle);

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(325, 5 + 20, 6));
        magnus_vertices.push_back(middle);

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(325, 5 + 30, 6));
        magnus_vertices.push_back(middle);

        end.makeStartOrEnd(Eigen::Vector3d(5, 5 + 30, 6), derivative_to_optimize);
        magnus_vertices.push_back(end);

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

        //Trajectory points :: Icarus

        start.makeStartOrEnd(Eigen::Vector3d(0, 79, 6), derivative_to_optimize);
        icarus_vertices.push_back(start);

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(325, 79, 6));
        icarus_vertices.push_back(middle);

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(325, 79 + 10, 6));
        icarus_vertices.push_back(middle);

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(5, 79 + 10, 6));
        icarus_vertices.push_back(middle);

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(5, 79 + 20, 6));
        icarus_vertices.push_back(middle);

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(325, 79 + 20, 6));
        icarus_vertices.push_back(middle);

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(325, 79 + 30, 6));
        icarus_vertices.push_back(middle);

        end.makeStartOrEnd(Eigen::Vector3d(5, 79 + 30, 6), derivative_to_optimize);
        icarus_vertices.push_back(end);



        std::vector<double> magnus_segment_times;
        std::vector<double> vega_segment_times;
        std::vector<double> icarus_segment_times;

        const double v_max = 5;
        const double a_max = 5;
        const double magic_fabian_constant = 6.5; // A tuning parameter.
        magnus_segment_times = estimateSegmentTimes(magnus_vertices, v_max, a_max);
        vega_segment_times = estimateSegmentTimes(vega_vertices, v_max, a_max);
        icarus_segment_times = estimateSegmentTimes(icarus_vertices, v_max, a_max);

        const int N = 10;
        mav_trajectory_generation::PolynomialOptimization<N> magnus_opt(dimension);
        mav_trajectory_generation::PolynomialOptimization<N> vega_opt(dimension);
        mav_trajectory_generation::PolynomialOptimization<N> icarus_opt(dimension);
    
        magnus_opt.setupFromVertices(magnus_vertices, magnus_segment_times, derivative_to_optimize);
        magnus_opt.solveLinear();

        vega_opt.setupFromVertices(vega_vertices, vega_segment_times, derivative_to_optimize);
        vega_opt.solveLinear();

        icarus_opt.setupFromVertices(icarus_vertices, icarus_segment_times, derivative_to_optimize);
        icarus_opt.solveLinear();



        mav_trajectory_generation::Segment::Vector magnus_segments;
        magnus_opt.getSegments(&magnus_segments);
        mav_trajectory_generation::Trajectory magnus_trajectory;
        magnus_opt.getTrajectory(&magnus_trajectory);

        mav_trajectory_generation::Segment::Vector vega_segments;
        vega_opt.getSegments(&vega_segments);
        mav_trajectory_generation::Trajectory vega_trajectory;
        vega_opt.getTrajectory(&vega_trajectory);

        mav_trajectory_generation::Segment::Vector icarus_segments;
        icarus_opt.getSegments(&icarus_segments);
        mav_trajectory_generation::Trajectory icarus_trajectory;
        icarus_opt.getTrajectory(&icarus_trajectory);

        double sampling_time = 2.0;
        int derivative_order = mav_trajectory_generation::derivative_order::POSITION;
        Eigen::VectorXd magnus_sample = magnus_trajectory.evaluate(sampling_time, derivative_order);
        Eigen::VectorXd vega_sample = vega_trajectory.evaluate(sampling_time, derivative_order);
        Eigen::VectorXd icarus_sample = icarus_trajectory.evaluate(sampling_time, derivative_order);

        double t_start = 2.0;
        double t_end = 10.0;
        double dt = 0.01;
        std::vector<Eigen::VectorXd> magnus_result;
        std::vector<Eigen::VectorXd> vega_result;
        std::vector<Eigen::VectorXd> icarus_result;
        std::vector<double> sampling_times; // Optional.
        magnus_trajectory.evaluateRange(t_start, t_end, dt, derivative_order, &magnus_result, &sampling_times);
        vega_trajectory.evaluateRange(t_start, t_end, dt, derivative_order, &vega_result, &sampling_times);
        icarus_trajectory.evaluateRange(t_start, t_end, dt, derivative_order, &icarus_result, &sampling_times);

        visualization_msgs::MarkerArray magnus_markers;
        visualization_msgs::MarkerArray vega_markers;
        visualization_msgs::MarkerArray icarus_markers;
        double distance = 1.0; // Distance by which to seperate additional markers. Set 0.0 to disable.
        std::string frame_id = "world";

        mav_trajectory_generation::drawMavTrajectory(magnus_trajectory, distance, frame_id, &magnus_markers);
        mav_planning_msgs::PolynomialTrajectory4D magnus_msg;
        mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(magnus_trajectory, &magnus_msg);
        magnus_msg.header.stamp = ros::Time::now();
        magnus_msg.header.frame_id = "world";
        polynomial_trajectory_pub_magnus.publish(magnus_msg);

        mav_trajectory_generation::drawMavTrajectory(vega_trajectory, distance, frame_id, &vega_markers);
        mav_planning_msgs::PolynomialTrajectory4D vega_msg;
        mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(vega_trajectory, &vega_msg);
        vega_msg.header.stamp = ros::Time::now();
        vega_msg.header.frame_id = "world";
        polynomial_trajectory_pub_vega.publish(vega_msg);

        mav_trajectory_generation::drawMavTrajectory(icarus_trajectory, distance, frame_id, &icarus_markers);
        mav_planning_msgs::PolynomialTrajectory4D icarus_msg;
        mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(icarus_trajectory, &icarus_msg);
        icarus_msg.header.stamp = ros::Time::now();
        icarus_msg.header.frame_id = "world";
        polynomial_trajectory_pub_icarus.publish(icarus_msg);


        ros::spinOnce();

        traj_pub_magnus.publish(magnus_markers);
        traj_pub_vega.publish(vega_markers);
        traj_pub_icarus.publish(icarus_markers);

        magnus_vertices.clear();
        vega_vertices.clear();
        icarus_vertices.clear();

        sleep_Rate.sleep();
    }
    return 0;
}