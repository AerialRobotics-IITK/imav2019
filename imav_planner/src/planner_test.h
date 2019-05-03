#include<ros/ros.h>
#include<nav_msgs/Odometry.h>
#include<geometry_msgs/PoseStamped.h>
#include<std_msgs/Bool.h>
#include<imav_planner/task_info.h>
#include<std_srvs/Empty.h>

#include <boost/msm/back/state_machine.hpp>
#include <boost/msm/back/mpl_graph_fsm_check.hpp>
#include <boost/msm/front/state_machine_def.hpp>
#include <boost/msm/front/functor_row.hpp>
#include <boost/msm/front/euml/common.hpp>
#include <boost/msm/front/euml/operator.hpp>

#define E 0.2

nav_msgs::Odometry mav_pose_;
imav_planner::task_info task_info_;
std_msgs::Bool gripper_status_;

void mav_pose_cb_(const nav_msgs::Odometry &msg)
{
    mav_pose_ = msg;
}

void task_info_cb_(const imav_planner::task_info &msg)
{
    task_info_ = msg;
}

void gripper_status_cb_(const std_msgs::Bool &msg)
{
    gripper_status_ = msg;
}

namespace state_machine 
{

    namespace msm = boost::msm;
    namespace mpl = boost::mpl;

    struct CmdTakeOff
    {
        ros::NodeHandle nh;
        CmdTakeOff(ros::NodeHandle nh_):nh(nh_){}
    };

    struct CmdTrajectory{};

    struct CmdGetPkg
    {
        ros::NodeHandle nh;
        CmdGetPkg(ros::NodeHandle nh_):nh(nh_){}
    };

    struct CmdDescent
    {
        ros::NodeHandle nh;
        CmdDescent(ros::NodeHandle nh_):nh(nh_){}
    };

    struct CmdGotoDrop
    {
        ros::NodeHandle nh;
        CmdGotoDrop(ros::NodeHandle nh_):nh(nh_){}
    };

    struct CmdAscent
    {
        ros::NodeHAndle nh;
        CmdAscent(ros::NodeHandle nh_):nh(nh_){}
    };

    struct CmdDrop{};

    struct CmdLand
    {
        ros::NodeHandle nh;
        CmdLand(ros::NodeHandle nh_):nh(nh_){}
    };

    struct CmdDropOver{};
    
    struct CmdHover
    {
        ros::NodeHandle nh;
        CmdHover(ros::NodeHandle nh_):nh(nh_){}
    };

    bool PkgAttached = false;
    bool ExploreDone = false;
    bool AtLZ = false;
    bool AtMailbox = false;

    struct test_fsm : public msm::front::state_machine_def <test_fsm>
    {
        template <class Event, class FSM>
        void on_entry(Event const&, FSM& )
        {
            std::cout<< "Entering FSM"<<std::endl;
        }

        template <class Event, class FSM>
        void on_exit(Event const &, FSM &)
        {
            std::cout << "Exiting FSM" << std::endl;
        }

        struct Rest : public msm::front::state <>
        {
            template <class Event, class FSM>
            void on_entry(Event const &, FSM &)
            {
                std::cout << "Entering Rest state" << std::endl;
            }

            template <class Event, class FSM>
            void on_exit(Event const &, FSM &)
            {
                std::cout << "Exiting Rest state" << std::endl;
            }
        };

        struct Hover : public msm::front::state<>
        {
            template <class Event, class FSM>
            void on_entry(Event const &, FSM &)
            {
                std::cout << "Entering Hover mode" << std::endl;
            }

            template <class Event, class FSM>
            void on_exit(Event const &, FSM &)
            {
                std::cout << "Exiting Hover mode" << std::endl;
            }
        };

        struct Explore : public msm::front::state<>
        {
            template <class Event, class FSM>
            void on_entry(Event const &, FSM &)
            {
                std::cout << "Entering Explore state" << std::endl;
            }

            template <class Event, class FSM>
            void on_exit(Event const &, FSM &)
            {
                std::cout << "Exiting Explore state" << std::endl;
            }
        };

        struct ReachLZ : public msm::front::state<>
        {
            template <class Event, class FSM>
            void on_entry(Event const &, FSM &)
            {
                std::cout << "Entering ReachingLZ state" << std::endl;
            }

            template <class Event, class FSM>
            void on_exit(Event const &, FSM &)
            {
                std::cout << "Exiting ReachingLZ state" << std::endl;
            }
        };

        struct ReachMailbox : public msm::front::state<>
        {
            template <class Event, class FSM>
            void on_entry(Event const &, FSM &)
            {
                std::cout << "Entering ReachingMailbox state" << std::endl;
            }

            template <class Event, class FSM>
            void on_exit(Event const &, FSM &)
            {
                std::cout << "Exiting Reaching Mailbox state" << std::endl;
            }
        };


        struct Descent : public msm::front::state<>
        {
            template <class Event, class FSM>
            void on_entry(Event const &, FSM &)
            {
                std::cout << "Entering Descent" << std::endl;
            }

            template <class Event, class FSM>
            void on_exit(Event const &, FSM &)
            {
                std::cout << "Exiting Descent" << std::endl;
            } 
        };

        struct Drop : public msm::front::state<>
        {
            template <class Event, class FSM>
            void on_entry(Event const &, FSM &)
            {
                std::cout << "Entering Drop state" << std::endl;
            }

            template <class Event, class FSM>
            void on_exit(Event const &, FSM &)
            {
                std::cout << "Exiting Drop state" << std::endl;
            }
        };

        typedef Rest initial_state;

        // void start_printing(print const &) { std::cout << "start_printet" << std::endl; }
        void TakeOff(CmdTakeOff const & cmd) 
        { 
            ros::NodeHandle nh_ = cmd.nh;
            ros::ServiceClient takeoff_client_ = nh_.serviceClient<std_srvs::Empty>("takeoff");
            ros::Subscriber odom_sub = nh_.subscribe("ground_truth/odometry", 1, mav_pose_cb_);
            while(mav_pose_.header.seq != 0){
                ros::spinOnce();
            }
            double start_height = mav_pose_.pose.pose.position.z;
            double target_height = start_height + 1.0;
            // might need to relax target_height to account for noise
            std_srvs::Empty takeoff_msg;
            if (takeoff_client_.call(takeoff_msg))
            {
                std::cout << "Taking off" << std::endl;
            }
            else
            {
                ROS_ERROR("Failed to takeoff");
            }
            while(mav_pose_.pose.pose.position.z < target_height){
                ros::spinOnce();
            }
        }

        void TrajExec(CmdTrajectory const &) { std::cout << "Exploring" << std::endl; ExploreDone=true;}

        void GetPkg(CmdGetPkg const & cmd) 
        {
            ros::NodeHandle nh_ = cmd.nh;
            ros::Subscriber task_info_sub = nh_.subscribe("task_info", 1000, task_info_cb_);
            ros::Publisher command_pub = nh_.advertise<geometry_msgs::PoseStamped>("command/pose", 1000);
            geometry_msgs::PoseStamped command_msg;
            while(task_info_.loc_type != "Home"){
                ros::spinOnce();
            }
            command_msg.header = task_info_.header;
            command_msg.pose.position = task_info_.position;
            int t = 0;
            // without this loop the topic doesn't seem to catch the command message
            while(t++<10){
                command_pub.publish(command_msg);
            }
        }

        void GotoDrop(CmdGotoDrop const & cmd) 
        { 
            ros::NodeHandle nh_ = cmd.nh;
            ros::Subscriber task_info_sub = nh_.subscribe("task_info", 1000, task_info_cb_);
            ros::Publisher command_pub = nh_.advertise<geometry_msgs::PoseStamped>("command/pose", 1000);
            geometry_msgs::PoseStamped command_msg;
            while(task_info_.loc_type != "Drop"){
                ros::spinOnce();
            }
            command_msg.header = task_info_.header;
            command_msg.pose.position = task_info_.position;
            int t = 0;
            // without this loop the topic doesn't seem to catch the command message
            while(t++<10){
                command_pub.publish(command_msg);
            }
            return;
        }

        void Ascending(CmdAscent const & cmd) 
        { 
            ros::NodeHandle nh_ = cmd.nh;
            // parametrize as hover height
            double target_height = 4.0;
            double curr_x, curr_y, curr_z=0;
            bool AscentDone = false;
            ros::Rate loopRate(10);
            ros::Subscriber mav_pose_sub = nh_.subscribe("ground_truth/odometry", 1, mav_pose_cb_);
            ros::Publisher command_pub = nh_.advertise<geometry_msgs::PoseStamped>("command/pose", 1);
            geometry_msgs::PoseStamped command_msg;
            while(curr_z==0)
            {
                ros::spinOnce();
                curr_x = mav_pose_.pose.pose.position.x;
                curr_y = mav_pose_.pose.pose.position.y;
                curr_z = mav_pose_.pose.pose.position.z;
                loopRate.sleep();
            }
            command_msg.header = mav_pose_.header;
            command_msg.pose.position.x = mav_pose_.pose.pose.position.x;
            command_msg.pose.position.y = mav_pose_.pose.pose.position.y;
            command_msg.pose.position.z = target_height;
            int t=0;
            while(!AscentDone)
            {
                if(t++<10){
                    command_pub.publish(command_msg);
                }
                ros::spinOnce();
                curr_z = mav_pose_.pose.pose.position.z;
                AscentDone = (curr_z < target_height) ? false : true;
                loopRate.sleep();
            }
            return;
        }

        void Dropping(CmdDrop const &) { std::cout << "Dropping" << std::endl; PkgAttached = false;}
        
        void Hovering(CmdHover const &) { std::cout << "Hovering" << std::endl; }
        
        void Landing(CmdLand const &) { 
            ros::NodeHandle nh_ = cmd.nh;
            // parametrize as land height
            double target_height = 0.08;
            double curr_x, curr_y, curr_z=0;
            bool LandingDone = false;
            ros::Rate loopRate(10);
            ros::Subscriber mav_pose_sub = nh_.subscribe("ground_truth/odometry", 1, mav_pose_cb_);
            ros::Publisher command_pub = nh_.advertise<geometry_msgs::PoseStamped>("command/pose", 1);
            geometry_msgs::PoseStamped command_msg;
            while(curr_z==0)
            {
                ros::spinOnce();
                curr_x = mav_pose_.pose.pose.position.x;
                curr_y = mav_pose_.pose.pose.position.y;
                curr_z = mav_pose_.pose.pose.position.z;
                loopRate.sleep();
            }
            command_msg.header = mav_pose_.header;
            command_msg.pose.position.x = mav_pose_.pose.pose.position.x;
            command_msg.pose.position.y = mav_pose_.pose.pose.position.y;
            command_msg.pose.position.z = target_height;
            int t=0;
            while(!LandingDone)
            {
                if(t++<10){
                    command_pub.publish(command_msg);
                }
                ros::spinOnce();
                curr_z = mav_pose_.pose.pose.position.z;
                LandingDone = (curr_z > target_height) ? false : true;
                loopRate.sleep();
            }
            PkgAttached = true;
            return;
        }
        
        void Descending(CmdDescent const & cmd) 
        { 
            ros::NodeHandle nh_ = cmd.nh;
            // parametrize as drop start height
            double target_height = 1.0;
            double curr_x, curr_y, curr_z=0;
            bool DescentDone = false;
            ros::Rate loopRate(10);
            ros::Subscriber mav_pose_sub = nh_.subscribe("ground_truth/odometry", 1, mav_pose_cb_);
            ros::Publisher command_pub = nh_.advertise<geometry_msgs::PoseStamped>("command/pose", 1);
            geometry_msgs::PoseStamped command_msg;
            while(curr_z==0)
            {
                ros::spinOnce();
                curr_x = mav_pose_.pose.pose.position.x;
                curr_y = mav_pose_.pose.pose.position.y;
                curr_z = mav_pose_.pose.pose.position.z;
                loopRate.sleep();
            }
            command_msg.header = mav_pose_.header;
            command_msg.pose.position.x = mav_pose_.pose.pose.position.x;
            command_msg.pose.position.y = mav_pose_.pose.pose.position.y;
            command_msg.pose.position.z = target_height;
            int t=0;
            while(!DescentDone)
            {
                if(t++<10){
                    command_pub.publish(command_msg);
                }

                ros::spinOnce();
                curr_z = mav_pose_.pose.pose.position.z;
                DescentDone = (curr_z > target_height) ? false : true;
                loopRate.sleep();
            }
            return;      
        }
        
        void DropOver(CmdDropOver const &) { std::cout << "Dropped Pkg" << std::endl; }
        

        bool isAtLoc(ros::NodeHandle nh)
        {
            ros::Subscriber mav_pose_sub = nh.subscribe("ground_truth/odometry", 1, mav_pose_cb_);
            ros::Rate loopRate(10);
            double curr_pos[3], target_pos[3];
            bool AtLoc = false;

            while(!AtLoc)
            {
                ros::spinOnce();
                double dist=0;

                curr_pos[0] = mav_pose_.pose.pose.position.x;
                curr_pos[1] = mav_pose_.pose.pose.position.y;
                curr_pos[2] = mav_pose_.pose.pose.position.z;

                target_pos[0] = task_info_.position.x;
                target_pos[1] = task_info_.position.y;
                target_pos[2] = task_info_.position.z;

                for(int i=0; i<3; i++)
                {
                    dist += (curr_pos[i] - target_pos[i])*(curr_pos[i] - target_pos[i]);
                }
                
                AtLoc = (dist > E*E) ? false : true;
                loopRate.sleep();
            }
            
            if(task_info_.loc_type=="Home")
            {
                AtLZ = AtLoc;
            }
            else if(task_info_.loc_type=="Drop")
            {
                AtMailbox = AtLoc;
            }

            return AtLoc;
        }

        bool ReachedLZ(CmdHover const & cmd)
        {
    
            if(isAtLoc(cmd.nh))
            {
                std::cout << "Reached LZ" << std::endl;
            }

            return AtLZ;
        }

        bool ReachedMailbox(CmdHover const & cmd)
        {
            if(isAtLoc(cmd.nh))
            {
                std::cout << "Reached Mailbox" << std::endl;
            }

            return AtMailbox;
        }

        template<class Event>
        bool NoPkg(Event const &){
            if(PkgAttached){
                std::cout << "Pkg is attached" << std::endl;
                return false;
            }
            else{
                std::cout << "Pkg is not attached" << std::endl;
                return true;
            }
        }

        template<class Event>
        bool HasPkg(Event const &){
            if(PkgAttached){
                std::cout << "Pkg is attached" << std::endl;
                return true;
            }
            else{
                std::cout << "Pkg is not attached" << std::endl;
                return false;
            }
        }

        bool StartExplore(CmdTrajectory const &){
            if(!ExploreDone){
                std::cout << "Exploration not finished" << std::endl;
                return true;
            }
            else{
                std::cout << "Exploration finished" << std::endl;
                return false;
            }
        }      
            
        bool FinExplore(CmdHover const &){
            if(!ExploreDone){
                std::cout << "Exploration not finished" << std::endl;
                return false;
            }
            else{
                std::cout << "Exploration finished" << std::endl;
                return true;
            }
        }

        typedef test_fsm tfsm;

        struct transition_table : mpl::vector<
                                    //    Start     Event         Next      Action				 Guard
                                    //  +---------+-------------+---------+---------------------+----------------------+
                                    a_row<Rest, CmdTakeOff, Hover, &tfsm::TakeOff>,
                                    //  +---------+-------------+---------+---------------------+----------------------+
                                    row<Hover, CmdTrajectory, Explore, &tfsm::TrajExec, &tfsm::StartExplore>,
                                    row<Hover, CmdGetPkg, ReachLZ, &tfsm::GetPkg, &tfsm::NoPkg>,
                                    row<Hover, CmdGotoDrop, ReachMailbox, &tfsm::GotoDrop, &tfsm::HasPkg>,
                                    a_row<Hover, CmdDescent, Descent, &tfsm::Descending>,
                                    //  +---------+-------------+---------+---------------------+----------------------+
                                    row<Descent, CmdLand, Rest, &tfsm::Landing, &tfsm::NoPkg>,
                                    row<Descent, CmdDrop, Drop, &tfsm::Dropping, &tfsm::HasPkg>,
                                    row<Descent, CmdAscent, Hover, &tfsm::Ascending, &tfsm::NoPkg>,
                                    //  +---------+-------------+---------+---------------------+----------------------+
                                    row<Drop, CmdDropOver, Descent, &tfsm::DropOver, &tfsm::NoPkg>,
                                    //  +---------+-------------+---------+---------------------+----------------------+
                                    row<ReachLZ, CmdHover, Hover, &tfsm::Hovering, &tfsm::ReachedLZ>,
                                    //  +---------+-------------+---------+---------------------+----------------------+
                                    row<ReachMailbox, CmdHover, Hover, &tfsm::Hovering, &tfsm::ReachedMailbox>,
                                    //  +---------+-------------+---------+---------------------+----------------------+
                                    row<Explore, CmdHover, Hover, &tfsm::Hovering, &tfsm::FinExplore>
                                    >
        {
        };
    };

    typedef msm::back::state_machine<test_fsm> test_fsm_;

    static char const *const state_names[] = {"Rest",
                                            "Hover",
                                            "Descent",
                                            "Drop",
                                            "ReachLZ",
                                            "ReachMailbox",
                                            "Explore"};

    void curr_state(test_fsm_ const& msg)
    {
        std::cout<<"Current state -- "<<state_names[msg.current_state()[0]]<<std::endl;
    }

}