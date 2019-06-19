#include<ros/ros.h>
#include<nav_msgs/Odometry.h>
#include<geometry_msgs/PoseStamped.h>
#include<std_msgs/Bool.h>
#include<imav_planner/task_info.h>
#include<std_srvs/Empty.h>
#include<std_msgs/UInt16.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <boost/msm/back/state_machine.hpp>
#include <boost/msm/back/mpl_graph_fsm_check.hpp>
#include <boost/msm/front/state_machine_def.hpp>
#include <boost/msm/front/functor_row.hpp>
#include <boost/msm/front/euml/common.hpp>
#include <boost/msm/front/euml/operator.hpp>

#define E 0.2

nav_msgs::Odometry mav_pose_;
imav_planner::task_info home_info_, drop_info_;
std_msgs::Bool gripper_status_;
std::string mode_;


void mav_pose_cb_(const nav_msgs::Odometry &msg)
{
    mav_pose_ = msg;
}

void home_info_cb_(const imav_planner::task_info &msg)
{
    home_info_ = msg;
}

void drop_info_cb_(const imav_planner::task_info &msg)
{
    drop_info_ = msg;
}

void gripper_status_cb_(const std_msgs::Bool &msg)
{
    gripper_status_ = msg;
}

void state_cb_(const mavros_msgs::State::ConstPtr &msg)
{
    mode_ = msg->mode;
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

    struct CmdTrajectory
    {
        ros::NodeHandle nh;
        CmdTrajectory(ros::NodeHandle nh_) : nh(nh_) {}
    };

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
        ros::NodeHandle nh;
        CmdAscent(ros::NodeHandle nh_):nh(nh_){}
    };

    struct CmdDrop
    {
        ros::NodeHandle nh;
        CmdDrop(ros::NodeHandle nh_):nh(nh_){}
    };

    struct CmdLand
    {
        ros::NodeHandle nh;
        CmdLand(ros::NodeHandle nh_):nh(nh_){}
    };

    struct CmdDropOver
    {
        ros::NodeHandle nh;
        CmdDropOver(ros::NodeHandle nh_):nh(nh_){}
    };
    
    struct CmdHover
    {
        ros::NodeHandle nh;
        CmdHover(ros::NodeHandle nh_):nh(nh_){}
    };

    bool PkgAttached = false;
    bool ExploreDone = false;
    bool AtLZ = false;
    bool AtMailbox = false;
    bool isSim = false;

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
            nh_.getParam("/isSim", isSim);
            ros::ServiceClient takeoff_client_ = nh_.serviceClient<std_srvs::Empty>("takeoff");
            ros::Subscriber odom_sub = nh_.subscribe("ground_truth/odometry", 1, mav_pose_cb_);
            ros::Publisher gripper_pub = nh_.advertise<std_msgs::UInt16>("servo", 1);
            
            std_msgs::UInt16 angle_msg;
            // when package is loaded angle is 140
            angle_msg.data = 140;
            int t=0;
            while(t++<10)
            {
                gripper_pub.publish(angle_msg);
            }
            std::cout << "Servo angle set to " << angle_msg.data << std::endl;

            double start_height=0;
            while(start_height == 0)
            {
                ros::spinOnce();
                start_height = mav_pose_.pose.pose.position.z;
            }

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

            while(mav_pose_.pose.pose.position.z < target_height)
            {
                ros::spinOnce();
            }

            Ascending(CmdAscent(nh_));
            PkgAttached = true;
            return;
        }

        void TrajExec(CmdTrajectory const & cmd) 
        {   
            std::cout << "Exploring" << std::endl; 
            
            ros::NodeHandle nh_ = cmd.nh;
            nh_.getParam("/isSim", isSim);
            ros::Subscriber state_sub_ = nh_.subscribe("pilot/state", 100, state_cb_);
            ros::Subscriber mav_pose_sub_ = nh_.subscribe("ground_truth/odometry", 100, mav_pose_cb_);
            ros::ServiceClient set_mode_client = nh_.serviceClient<mavros_msgs::SetMode>("pilot/set_mode");
            ros::Subscriber drop_sub_ = nh_.subscribe("drop_info", 100, drop_info_cb_);

            mavros_msgs::SetMode mission_set_mode, offb_set_mode;
            offb_set_mode.request.custom_mode = "OFFBOARD";
            mission_set_mode.request.custom_mode = "AUTO.MISSION";
            bool mode_set_= false;

            while (ros::ok()&&(!mode_set_))
            {
                ros::spinOnce();
                if(mode_=="OFFBOARD")
                {
                    if (set_mode_client.call(mission_set_mode) && mission_set_mode.response.mode_sent)
                    {
                        std::cout << "  Mission 1 enabled"<<std::endl;
                        mode_set_=true;
                    }
                }
            }

            drop_info_.loc_type = "";
            while (drop_info_.loc_type != "Drop")
            {
                ros::spinOnce();
                if(mav_pose_.pose.pose.position.x < 0.5 && mav_pose_.pose.pose.position.y < 0.5)
                {
                    while (mode_set_)
                    {
                        ros::spinOnce();
                        if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                        {
                            std::cout << "  Offboard 1 enabled" << std::endl;
                            mode_set_ = false;
                        }
                    }
                    ExploreDone = true;
                    return;
                }
            }

            while (mode_set_)
            {
                ros::spinOnce();
                if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                {
                    std::cout << "  Offboard 2 enabled" << std::endl;
                    mode_set_ = false;
                }
            }

            GotoDrop(CmdGotoDrop(nh_));
            if(ReachedMailbox(CmdHover(nh_)))
            {
                Descending(CmdDescent(nh_));
                Dropping(CmdDrop(nh_));
                DropOver(CmdDropOver(nh_));
                Ascending(CmdAscent(nh_));
            }

            while (ros::ok() && (!mode_set_))
            {
                ros::spinOnce();
                if (mode_ == "OFFBOARD")
                {
                    if (set_mode_client.call(mission_set_mode) && mission_set_mode.response.mode_sent)
                    {
                        std::cout << "  Mission 2 enabled" << std::endl;
                        mode_set_ = true;
                    }
                }
            }

            while(mav_pose_.pose.pose.position.x > 0.5 || mav_pose_.pose.pose.position.y > 0.5)
            {
                ros::spinOnce();
            }

            while (mode_set_)
            {
                ros::spinOnce();
                if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                {
                    std::cout << "  Offboard 3 enabled" << std::endl;
                    mode_set_ = false;
                }
            }

            ExploreDone = true;
            return;
        } 

        void GetPkg(CmdGetPkg const & cmd) 
        {
            std::cout << "Going to LZ" << std::endl;

            ros::NodeHandle nh_ = cmd.nh;
            ros::Subscriber home_info_sub = nh_.subscribe("home_info", 1000, home_info_cb_);
            ros::Publisher command_pub = nh_.advertise<geometry_msgs::PoseStamped>("command/pose", 1000);
            geometry_msgs::PoseStamped command_msg;
            home_info_.loc_type = "";
            while(home_info_.loc_type != "Home")
            {
                ros::spinOnce();
            }
            command_msg.header = home_info_.header;
            command_msg.pose.position.x = home_info_.position.x;
            command_msg.pose.position.y = home_info_.position.y;
            
            double hover_height=4.0;
            nh_.getParam("/hoverHeight", hover_height);
            command_msg.pose.position.z = hover_height;

            int t = 0; 
            // without this loop the topic doesn't seem to catch the command message
            while(t++<1000)
            {
                command_pub.publish(command_msg);
            }
            return;
        }

        void GotoDrop(CmdGotoDrop const & cmd) 
        { 
            std::cout << "Going to Mailbox" << std::endl;

            ros::NodeHandle nh_ = cmd.nh;
            ros::Subscriber drop_info_sub = nh_.subscribe("drop_info", 1000, drop_info_cb_);
            ros::Publisher command_pub = nh_.advertise<geometry_msgs::PoseStamped>("command/pose", 1000);
            
            geometry_msgs::PoseStamped command_msg;
            drop_info_.loc_type = "";
            while(drop_info_.loc_type != "Drop")
            {
                ros::spinOnce();
            }
            command_msg.header = drop_info_.header;
            command_msg.pose.position.x = drop_info_.position.x;
            command_msg.pose.position.y = drop_info_.position.y;
                        
            double hover_height=4.0;
            nh_.getParam("/hoverHeight", hover_height);
            command_msg.pose.position.z = hover_height;
            
            int t = 0;
            // without this loop the topic doesn't seem to catch the command message
            while(t++<1000)
            {
                command_pub.publish(command_msg);
            }
            return;
        }

        void Ascending(CmdAscent const & cmd) 
        { 
            std::cout << "Ascending" << std::endl;

            ros::NodeHandle nh_ = cmd.nh;
            
            double hover_height=4.0;
            nh_.getParam("/hoverHeight", hover_height);
            // std::cout<<hover_height<<std::endl;
            nh_.getParam("/isSim", isSim);

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
            command_msg.pose.position.z = hover_height;
            
            int t=0;
            while(!AscentDone)
            {
                if(t++<10){
                    command_pub.publish(command_msg);
                }
                ros::spinOnce();
                curr_z = mav_pose_.pose.pose.position.z;
                AscentDone = (curr_z < hover_height) ? false : true;
                loopRate.sleep();
            }
            return;
        }

        void Dropping(CmdDrop const & cmd)
        { 
            std::cout << "Dropping Package" << std::endl;

            ros::NodeHandle nh_ = cmd.nh;
            ros::Publisher gripper_pub = nh_.advertise<std_msgs::UInt16>("servo", 1);
            std_msgs::UInt16 angle_msg;
            angle_msg.data = 40;
            int t=0;
            while(t++<10)
            {
                gripper_pub.publish(angle_msg);
            }
            std::cout << "Servo angle set to " << angle_msg.data << std::endl;
            PkgAttached = false;
        }

        void Hovering(CmdHover const & cmd) 
        { 
            std::cout << "Hovering" << std::endl;

            ros::NodeHandle nh_ = cmd.nh;
            double sleep_rate=0.2, hold_interval=5;
            nh_.getParam("/holdInterval", hold_interval);
            sleep_rate = 1/hold_interval;
            ros::Rate loopRate(sleep_rate);
            loopRate.sleep();
        }
        
        void Landing(CmdLand const & cmd) 
        { 
            std::cout << "Landing" << std::endl;

            ros::NodeHandle nh_ = cmd.nh;
            double land_height=0.08;
            nh_.getParam("/landHeight", land_height);
            nh_.getParam("/isSim", isSim);
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
            command_msg.pose.position.z = land_height;
            int t=0;
            while(!LandingDone)
            {
                if(t++<10)
                {
                    command_pub.publish(command_msg);
                }
                ros::spinOnce();
                curr_z = mav_pose_.pose.pose.position.z;
                LandingDone = (curr_z > land_height) ? false : true;
                loopRate.sleep();
            }
            PkgAttached = true;
            return;
        }
        
        void Descending(CmdDescent const & cmd) 
        { 
            std::cout << "Descending" << std::endl;

            ros::NodeHandle nh_ = cmd.nh;
            double descent_height=0.3;
            nh_.getParam("/descentHeight", descent_height);
            nh_.getParam("/isSim", isSim);

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
            command_msg.pose.position.z = descent_height;
            int t=0;
            while(!DescentDone)
            {
                if(t++<10){
                    command_pub.publish(command_msg);
                }

                ros::spinOnce();
                curr_z = mav_pose_.pose.pose.position.z;
                DescentDone = (curr_z > descent_height) ? false : true;
                loopRate.sleep();
            }

            return;      
        }
        
        void DropOver(CmdDropOver const & cmd) 
        {
            std::cout << "Package Dropped" << std::endl;

            ros::NodeHandle nh_ = cmd.nh;
            ros::Publisher gripper_pub = nh_.advertise<std_msgs::UInt16>("servo", 1);
            std_msgs::UInt16 angle_msg;
            angle_msg.data = 90;
            int t=0;
            while(t++<10)
            {
                gripper_pub.publish(angle_msg);
            }
            std::cout << "Servo angle set to " << angle_msg.data << std::endl;
            PkgAttached = false;
        }

        bool isAtLZ(ros::NodeHandle nh)
        {
            nh.getParam("/isSim", isSim);
            double hover_height = 4.0;
            nh.getParam("/hoverHeight", hover_height);
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

                target_pos[0] = home_info_.position.x;
                target_pos[1] = home_info_.position.y;
                target_pos[2] = hover_height;

                for(int i=0; i<3; i++)
                {
                    dist += (curr_pos[i] - target_pos[i])*(curr_pos[i] - target_pos[i]);
                }
                
                AtLoc = (dist > E*E) ? false : true;
                loopRate.sleep();
            }

            AtLZ=AtLoc;
            return AtLoc;
        }

        bool isAtMB(ros::NodeHandle nh)
        {
            nh.getParam("/isSim", isSim);
            double hover_height = 4.0;
            nh.getParam("/hoverHeight", hover_height);
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

                target_pos[0] = drop_info_.position.x;
                target_pos[1] = drop_info_.position.y;
                target_pos[2] = hover_height;

                for(int i=0; i<3; i++)
                {
                    dist += (curr_pos[i] - target_pos[i])*(curr_pos[i] - target_pos[i]);
                }
                
                AtLoc = (dist > E*E) ? false : true;
                loopRate.sleep();
            }
            
            AtMailbox=AtLoc;
            return AtLoc;
        }

        bool ReachedLZ(CmdHover const & cmd)
        {
            if(isAtLZ(cmd.nh))
            {
                std::cout << "Reached LZ" << std::endl;
            }

            return AtLZ;
        }

        bool ReachedMailbox(CmdHover const & cmd)
        {
            if(isAtMB(cmd.nh))
            {
                std::cout << "Reached Mailbox" << std::endl;
            }

            return AtMailbox;
        }

        template<class Event>
        bool NoPkg(Event const &){
            if(PkgAttached)
            {
                std::cout << "Pkg is attached, cannot proceed" << std::endl;
                return false;
            }
            else
            {
                std::cout << "Pkg is not attached, proceeding" << std::endl;
                return true;
            }
        }

        template<class Event>
        bool HasPkg(Event const &){
            if(PkgAttached)
            {
                std::cout << "Pkg is attached, proceeding" << std::endl;
                return true;
            }
            else
            {
                std::cout << "Pkg is not attached, cannot proceed" << std::endl;
                return false;
            }
        }

        bool StartExplore(CmdTrajectory const &){
            if(!ExploreDone)
            {
                std::cout << "Exploration not finished, starting" << std::endl;
                return true;
            }
            else
            {
                std::cout << "Exploration finished, skipping" << std::endl;
                return false;
            }
        }      
            
        bool FinExplore(CmdHover const &){
            if(!ExploreDone)
            {
                std::cout << "Exploration not finished, cannot proceed" << std::endl;
                return false;
            }
            else
            {
                std::cout << "Exploration finished, proceeding" << std::endl;
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