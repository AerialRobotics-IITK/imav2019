#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <mav_utils_msgs/TaskInfo.h>
#include <std_srvs/Empty.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mav_utils_msgs/signal.h>
#include <future>

#include <boost/msm/back/state_machine.hpp>
#include <boost/msm/back/mpl_graph_fsm_check.hpp>
#include <boost/msm/front/state_machine_def.hpp>
#include <boost/msm/front/functor_row.hpp>
#include <boost/msm/front/euml/common.hpp>
#include <boost/msm/front/euml/operator.hpp>

/* 
    further progress
    -- update exit mission condition
    -- remove home_info and replace with planned home (pilot/home_location/home)
    -- set dynamic reconfigure for height and angle params
    -- check rates and queues for pub-sub objects
    -- add error margin for all position data comparisons
    -- would global pubbers/subbers work?
                                            */


// squared error for location comparison
#define E 0.6
#define echo(X) std::cout << X << std::endl;

// storage variables
nav_msgs::Odometry mav_pose_;
mav_utils_msgs::TaskInfo drop_info_, home_info_;
std_msgs::Bool gripper_status_;
std::string mode_;

// height params - tuned for tfmini
double hover_height = 4.0;
double drop_height = 1.2;
double land_height = 0.41;

// servo angles
double open_angle = 40;
double close_angle = 140;
double eq_angle = 90;

// Rates
double hover_time = 5.0;
double transition_time = 5.0;
double wait_time = 20.0;

// callback functions

void mav_pose_cb_(const nav_msgs::Odometry &msg)
{
    mav_pose_ = msg;
}

void home_info_cb_(const mav_utils_msgs::TaskInfo &msg)
{
    home_info_ = msg;
}

void drop_info_cb_(const mav_utils_msgs::TaskInfo &msg)
{
    drop_info_ = msg;
    // echo("Drop location: x = " << drop_info_.position.x << ", y = " << drop_info_.position.y);
}

void state_cb_(const mavros_msgs::State::ConstPtr &msg)
{
    mode_ = msg->mode;
}

// state machine definition

namespace state_machine
{
    namespace msm = boost::msm;
    namespace mpl = boost::mpl;

    // state machine commands

    struct CmdTakeOff
    {
        ros::NodeHandle nh;
        CmdTakeOff(ros::NodeHandle nh_):nh(nh_){}
    };

    struct CmdExploring
    {
        ros::NodeHandle nh;
        CmdExploring(ros::NodeHandle nh_) : nh(nh_) {}
    };

    struct CmdGotoLZ
    {
        ros::NodeHandle nh;
        CmdGotoLZ(ros::NodeHandle nh_):nh(nh_){}
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

    // state variables

    bool PkgAttached = true;
    bool AtLZ = false;
    bool AtMailbox = false;
    bool ContMission = true;

    // state machine object

    struct fsm :  public msm::front::state_machine_def <fsm>
    {
        typedef msm::active_state_switch_before_transition active_state_switch_policy;

        template <class Event, class FSM>
        void on_entry(Event const&, FSM& )
        {
            echo("Entered state machine");
        }

        template <class Event, class FSM>
        void on_exit(Event const &, FSM &)
        {
            echo("Exited state machine");
        }

        // state definitions

        struct Rest : public msm::front::state <>
        {
            template <class Event, class FSM>
            void on_entry(Event const &, FSM &)
            {
                echo("Entered Rest state");
            }

            template <class Event, class FSM>
            void on_exit(Event const &, FSM &)
            {
                echo("Exited Rest state");
            }
        };

        struct Hover : public msm::front::state<>
        {
            template <class Event, class FSM>
            void on_entry(Event const &, FSM &)
            {
                echo("Entered Hover mode");
            }

            template <class Event, class FSM>
            void on_exit(Event const &, FSM &)
            {
                echo("Exited Hover mode");
            }
        };

        struct Exploring : public msm::front::state<>
        {
            template <class Event, class FSM>
            void on_entry(Event const &, FSM &)
            {
                echo("Entered Exploring state");
            }

            template <class Event, class FSM>
            void on_exit(Event const &, FSM &)
            {
                echo("Exited Exploring state");
            }
        };

        struct ReachLZ : public msm::front::state<>
        {
            template <class Event, class FSM>
            void on_entry(Event const &, FSM &)
            {
                echo("Entered ReachingLZ state");
            }

            template <class Event, class FSM>
            void on_exit(Event const &, FSM &)
            {
                echo("Exited ReachingLZ state");
            }
        };

        struct ReachMailbox : public msm::front::state<>
        {
            template <class Event, class FSM>
            void on_entry(Event const &, FSM &)
            {
                echo("Entered Reaching_Mailbox state");
            }

            template <class Event, class FSM>
            void on_exit(Event const &, FSM &)
            {
                echo("Exited Reaching_Mailbox state");
            }
        };

        struct Descent : public msm::front::state<>
        {
            template <class Event, class FSM>
            void on_entry(Event const &, FSM &)
            {
                echo("Entered Descent");
            }

            template <class Event, class FSM>
            void on_exit(Event const &, FSM &)
            {
                echo("Exited Descent");
            } 
        };

        struct Drop : public msm::front::state<>
        {
            template <class Event, class FSM>
            void on_entry(Event const &, FSM &)
            {
                echo("Entered Drop state");
            }

            template <class Event, class FSM>
            void on_exit(Event const &, FSM &)
            {
                echo("Exited Drop state");
            }
        };

        // initial state declaration
        typedef Rest initial_state;

        // state transition functions

        void TakeOff(CmdTakeOff const & cmd)
        {
            echo(" Starting execution");

            ros::NodeHandle nh_ = cmd.nh;
            ros::Rate loopRate(10);  
            ros::Rate sleepRate(1);
            // ros::ServiceClient takeoff_client_ = nh_.serviceClient<std_srvs::Empty>("takeoff");
            ros::Subscriber odom_sub_ = nh_.subscribe("odometry", 10, mav_pose_cb_);
            ros::Publisher gripper_pub_ = nh_.advertise<std_msgs::Bool>("servo", 1);
            // ros::Publisher command_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("command/pose", 10);

            // std_msgs::Bool angle_msg;
            // echo("  Setting servo angle");
            // angle_msg.data = close_angle;
            echo("   Publishing servo command");
            std_msgs::Bool angle_msg;
            angle_msg.data = true;
            gripper_pub_.publish(angle_msg);
            echo("   Published servo command");
            
            echo("  Servo angle set to " << (int) angle_msg.data);

            double start_height=0;
            echo("  Waiting for first odometry message");
            while(start_height == 0)
            {
                ros::spinOnce();
                start_height = mav_pose_.pose.pose.position.z;
                loopRate.sleep();
            }
            echo("  Got first odometry message");

            // double target_height = start_height + 1.0;
            // // might need to relax target_height to account for noise
            // std_srvs::Empty takeoff_msg;
            // if (takeoff_client_.call(takeoff_msg))
            // {
            //     echo("  Taking off");
            // }
            // else
            // {
            //     ROS_ERROR("Failed to takeoff");
            // }

            // while(mav_pose_.pose.pose.position.z < target_height)
            // {
            //     ros::spinOnce();
            //     loopRate.sleep();
            // }

            // echo("  Takeoff achieved");

            // double curr_x, curr_y, curr_z=0;
            // bool AscentDone = false;

            // geometry_msgs::PoseStamped command_msg;
            // echo("  Waiting for odometry");
            // while(curr_z==0)
            // {
            //     ros::spinOnce();
            //     curr_x = mav_pose_.pose.pose.position.x;
            //     curr_y = mav_pose_.pose.pose.position.y;
            //     curr_z = mav_pose_.pose.pose.position.z;
            //     loopRate.sleep();
            // }
            // echo("  Received odometry");

            // command_msg.header.stamp = ros::Time::now();
            // // command_msg.header.frame_id = "map";
            // command_msg.pose.position.x = mav_pose_.pose.pose.position.x;
            // command_msg.pose.position.y = mav_pose_.pose.pose.position.y;
            // command_msg.pose.position.z = hover_height;
            // command_msg.pose.orientation = mav_pose_.pose.pose.orientation;

            // echo("  Starting ascent");
            
            // sleepRate.sleep();
            // command_pub_.publish(command_msg);

            // while(!AscentDone)
            // {
            //     ros::spinOnce();
            //     curr_z = mav_pose_.pose.pose.position.z; 
            //     AscentDone = (curr_z < hover_height) ? false : true;
            //     loopRate.sleep();
            // }
            // echo("  Ascent done");

            PkgAttached = true;
            return;
        }

        void Explore(CmdExploring const & cmd) 
        {
            echo(" Exploring");

            ros::NodeHandle nh_ = cmd.nh;
            ros::Rate loopRate(10);  
            ros::Rate sleepRate(1);
            ros::Subscriber state_sub_ = nh_.subscribe("pilot/state", 1, state_cb_);
            ros::Subscriber mav_pose_sub_ = nh_.subscribe("odometry", 10, mav_pose_cb_);
            ros::ServiceClient set_mode_client = nh_.serviceClient<mavros_msgs::SetMode>("pilot/set_mode");
            ros::Subscriber drop_info_sub_ = nh_.subscribe("drop_info", 1, drop_info_cb_);
            ros::Publisher command_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("command/pose", 10);
            ros::ServiceClient node_client = nh_.serviceClient<mav_utils_msgs::signal>("detector/terminate");

            mav_utils_msgs::signal start;
            start.request.signal = 1;
            if(node_client.call(start) && start.response.success)
            {
                echo("  Started detector node");
            }

            mavros_msgs::SetMode mission_set_mode, offb_set_mode;
            offb_set_mode.request.custom_mode = "OFFBOARD";
            mission_set_mode.request.custom_mode = "AUTO.MISSION";
            bool mode_set_= false;

            echo("  Changing mode to Mission");                                                           // Add state check
            while (!mode_set_)
            {
                ros::spinOnce();
                // if(mode_=="OFFBOARD")
                // {
                    if (set_mode_client.call(mission_set_mode) && mission_set_mode.response.mode_sent)
                    {
                        echo("   Mission enabled");
                        mode_set_=true;
                    }
                // }
                loopRate.sleep();
            }
            echo("  Changed mode to Mission");

            geometry_msgs::PoseStamped command_msg;
            drop_info_.loc_type = "";
            echo("  Searching for mailboxes");
            while (drop_info_.loc_type != "Drop")
            {
                ros::spinOnce();

                // exit condition
                if(mav_pose_.pose.pose.position.x < -0.5 && mav_pose_.pose.pose.position.y < -0.5)            // LZ positions
                {
                    echo("   Reached LZ, mission over.")
				    PkgAttached=false;
                    if(!PkgAttached)
                    {
                        echo("   No package attached, switching from Mission mode");
                        while (mode_set_)
                        {
                            double curr_x, curr_y, curr_z=0;
                            echo("   Waiting for odometry");
                            while(curr_z==0)
                            {
                                ros::spinOnce();
                                curr_x = mav_pose_.pose.pose.position.x;
                                curr_y = mav_pose_.pose.pose.position.y;
                                curr_z = mav_pose_.pose.pose.position.z;
                                loopRate.sleep();
                            }
                            echo("   Received odometry");

                            command_msg.header.stamp = ros::Time::now();
                            // command_msg.header.frame_id = "map";
                            command_msg.pose.position.x = curr_x;
                            command_msg.pose.position.y = curr_y;
                            command_msg.pose.position.z = curr_z;
                            command_msg.pose.orientation = mav_pose_.pose.pose.orientation;

                            sleepRate.sleep();
                            command_pub_.publish(command_msg);

                            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                            {
                                echo("    Offboard enabled");
                                mode_set_ = false;
                            }
                            loopRate.sleep();
                        }
                        echo("   Switched to Offboard");

                        ContMission = false;
                        return;
                    }
                }
            }

            echo("  Mailbox detected, switching from Mission mode");
            echo("   Drop location: x = " << drop_info_.position.x << ", y = " << drop_info_.position.y);
            drop_info_sub_.shutdown();
            // echo("   Drop info subscriber shutdown");
            // echo("   Drop location: x = " << drop_info_.position.x << ", y = " << drop_info_.position.y);

            while (mode_set_)
            {
                double curr_x, curr_y, curr_z=0;
                echo("   Waiting for odometry");
                while(curr_z==0)
                {
                    ros::spinOnce();
                    curr_x = mav_pose_.pose.pose.position.x;
                    curr_y = mav_pose_.pose.pose.position.y;
                    curr_z = mav_pose_.pose.pose.position.z;
                    loopRate.sleep();
                }

                command_msg.header.stamp = ros::Time::now();
                // command_msg.header.frame_id = "map";
                command_msg.pose.position.x = curr_x;
                command_msg.pose.position.y = curr_y;
                command_msg.pose.position.z = curr_z;
                command_msg.pose.orientation = mav_pose_.pose.pose.orientation;
                
                sleepRate.sleep();
                command_pub_.publish(command_msg);

                if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                {
                    echo("   Offboard enabled");
                    mode_set_ = false;
                }
                loopRate.sleep();
            }
            echo("  Switched to Offboard");

            return;
        }

        void GotoDrop(CmdGotoDrop const & cmd)
        {
            echo(" Going to Mailbox");

            ros::NodeHandle nh_ = cmd.nh;
            ros::Rate loopRate(10);  
            ros::Rate sleepRate(1);
            // ros::Subscriber drop_info_sub_ = nh_.subscribe("drop_info", 1, drop_info_cb_);
            ros::Subscriber mav_pose_sub_ = nh_.subscribe("odometry", 10, mav_pose_cb_);
            ros::Publisher command_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("command/pose", 50);
            
            geometry_msgs::PoseStamped command_msg;
            // drop_info_.loc_type = "";
            // edit later
            echo("  Checking for drop location");
            if(drop_info_.loc_type != "Drop"){ 
                echo("  Waiting for drop location");
                while(drop_info_.loc_type != "Drop")
                {
                    ros::spinOnce();
                    loopRate.sleep();
                }
            }
            echo("  Received drop location");
            
            command_msg.header.stamp = ros::Time::now();
            // command_msg.header.frame_id = "map";
            command_msg.pose.position.x = drop_info_.position.x;
            command_msg.pose.position.y = drop_info_.position.y;
            command_msg.pose.position.z = hover_height;
            command_msg.pose.orientation = mav_pose_.pose.pose.orientation;
            echo("   Command location: x = " << command_msg.pose.position.x << ", y = " << command_msg.pose.position.y);


            sleepRate.sleep();
            command_pub_.publish(command_msg);
            echo("   published once")

            int t=0;
            while(t<1000)
            {
                command_pub_.publish(command_msg);
                t++;
            }
            echo("   published recursively");

            return;
        }

        void Descending(CmdDescent const & cmd)
        {
            echo(" Descending");
            ros::NodeHandle nh_ = cmd.nh;
            ros::Rate loopRate(10);  
            ros::Rate sleepRate(1);
            ros::Subscriber mav_pose_sub_ = nh_.subscribe("odometry", 10, mav_pose_cb_);
            ros::Publisher command_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("command/pose", 10);

            double curr_x, curr_y, curr_z=0;
            bool DescentDone = false;
            geometry_msgs::PoseStamped command_msg;
            
            echo("  Waiting for odometry");
            while(curr_z==0)
            {
                ros::spinOnce();
                curr_x = mav_pose_.pose.pose.position.x;
                curr_y = mav_pose_.pose.pose.position.y;
                curr_z = mav_pose_.pose.pose.position.z;
                loopRate.sleep();
            }
            echo("  Received odometry");

            command_msg.header.stamp = ros::Time::now();
            // command_msg.header.frame_id = "map";
            command_msg.pose.position.x = mav_pose_.pose.pose.position.x;
            command_msg.pose.position.y = mav_pose_.pose.pose.position.y;
            command_msg.pose.position.z = drop_height;
            command_msg.pose.orientation = mav_pose_.pose.pose.orientation;
            // echo("Command height :" << command_msg.pose.position.z);
            
            echo("  Starting descent");
            
            sleepRate.sleep();
            command_pub_.publish(command_msg);

            while(!DescentDone)
            {
                ros::spinOnce();
                curr_z = mav_pose_.pose.pose.position.z;
                DescentDone = (curr_z > drop_height) ? false : true;
                loopRate.sleep();
            }
            echo("  Descent done");

            return;      
        }

        void GotoLZ(CmdGotoLZ const & cmd)
        {
            echo(" Going to LZ");

            ros::NodeHandle nh_ = cmd.nh;
            ros::Rate loopRate(10);  
            ros::Rate sleepRate(1);
            ros::Subscriber home_info_sub_ = nh_.subscribe("home_info", 1, home_info_cb_);
            ros::Subscriber mav_pose_sub_ = nh_.subscribe("odometry", 10, mav_pose_cb_);
            ros::Publisher command_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("command/pose", 10);

            geometry_msgs::PoseStamped command_msg;
            home_info_.loc_type = "";
            echo("  Waiting for Home location");
            while(home_info_.loc_type != "Home")
            {
                ros::spinOnce();
                loopRate.sleep();
            }
            echo("  Home location received");

            command_msg.header.stamp = ros::Time::now();
            // command_msg.header.frame_id = "map";
            command_msg.pose.position.x = home_info_.position.x;
            command_msg.pose.position.y = home_info_.position.y;
            command_msg.pose.position.z = hover_height;
            command_msg.pose.orientation = mav_pose_.pose.pose.orientation;
            
            sleepRate.sleep();
            command_pub_.publish(command_msg);

            return;
        }

        void Hovering(CmdHover const & cmd)
        {
            echo(" Hovering");

            ros::Rate hoverRate(1.0/hover_time);
            hoverRate.sleep();            
        }

        void Dropping(CmdDrop const & cmd)
        {
            echo(" Dropping Package");

            ros::NodeHandle nh_ = cmd.nh;
            ros::Rate loopRate(10);  
            ros::Rate sleepRate(1);
            ros::Publisher gripper_pub_ = nh_.advertise<std_msgs::Bool>("servo", 1);
            
            // std_msgs::UInt16 angle_msg;
            // echo("  Setting servo angle");
            // angle_msg.data = open_angle;

            std_msgs::Bool angle_msg;
            angle_msg.data = false;
            gripper_pub_.publish(angle_msg);
            
            echo("  Servo angle set to " << angle_msg.data);
            
            PkgAttached = false;
            return;
        }

        void Ascending(CmdAscent const & cmd)
        {
            echo(" Ascending");

            ros::NodeHandle nh_ = cmd.nh;
            ros::Rate loopRate(10);  
            ros::Rate sleepRate(1);
            ros::Subscriber mav_pose_sub_ = nh_.subscribe("odometry", 10, mav_pose_cb_);
            ros::Publisher command_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("command/pose", 10);
            
            double curr_x, curr_y, curr_z=0;
            bool AscentDone = false;

            geometry_msgs::PoseStamped command_msg;
            echo("  Waiting for odometry");
            while(curr_z==0)
            {
                ros::spinOnce();
                curr_x = mav_pose_.pose.pose.position.x;
                curr_y = mav_pose_.pose.pose.position.y;
                curr_z = mav_pose_.pose.pose.position.z;
                loopRate.sleep();
            }
            echo("  Received odometry");

            command_msg.header.stamp = ros::Time::now();
            // command_msg.header.frame_id = "map";
            command_msg.pose.position.x = mav_pose_.pose.pose.position.x;
            command_msg.pose.position.y = mav_pose_.pose.pose.position.y;
            command_msg.pose.position.z = hover_height;
            command_msg.pose.orientation = mav_pose_.pose.pose.orientation;

            echo("  Starting ascent");

            sleepRate.sleep();
            command_pub_.publish(command_msg);
            
            while(!AscentDone)
            {
                ros::spinOnce();
                curr_z = mav_pose_.pose.pose.position.z;
                AscentDone = (curr_z < hover_height) ? false : true;
                loopRate.sleep();
            }
            echo("  Ascent done");

            return;
        }

        void Landing(CmdLand const & cmd)
        {
            echo(" Landing");

            ros::NodeHandle nh_ = cmd.nh;
            ros::Rate loopRate(10);  
            ros::Rate sleepRate(1);
            ros::Subscriber mav_pose_sub_ = nh_.subscribe("odometry", 10, mav_pose_cb_);
            ros::Subscriber heli_sub_ = nh_.subscribe("drop_info", 10, drop_info_cb_);
            ros::Publisher command_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("command/pose", 10);
            ros::ServiceClient detector_client = nh_.serviceClient<mav_utils_msgs::signal>("detector/terminate");
            ros::ServiceClient helipad_client = nh_.serviceClient<mav_utils_msgs::signal>("hdetect/terminate");

            mav_utils_msgs::signal stop, start;
            stop.request.signal = 0;
            start.request.signal = 1;
            if(detector_client.call(stop) && stop.response.success)
            {
                echo("  detector node stopped");
            }
            if(helipad_client.call(start) && start.response.success)
            {
                echo("  hdetect node started");
            }

            double curr_x, curr_y, curr_z=0;
            bool LandingDone = false;
            
            geometry_msgs::PoseStamped command_msg;
            echo("  Waiting for odometry");
            while(curr_z==0)
            {
                ros::spinOnce();
                curr_x = mav_pose_.pose.pose.position.x;
                curr_y = mav_pose_.pose.pose.position.y;
                curr_z = mav_pose_.pose.pose.position.z;
                loopRate.sleep();
            }
            echo("  Received odometry");

            command_msg.header.stamp = ros::Time::now();
            // command_msg.header.frame_id = "map";
            command_msg.pose.position.x = mav_pose_.pose.pose.position.x;
            command_msg.pose.position.y = mav_pose_.pose.pose.position.y;
            command_msg.pose.position.z = land_height;
            command_msg.pose.orientation = mav_pose_.pose.pose.orientation;

            echo("  Landing initiated");

            sleepRate.sleep();
            command_pub_.publish(command_msg);

            while(!LandingDone)
            {
                ros::spinOnce();
                curr_z = mav_pose_.pose.pose.position.z;
                LandingDone = (curr_z > land_height) ? false : true;
                loopRate.sleep();
            }
            echo("  Landing done");

            return;
        }

        void DropOver(CmdDropOver const & cmd)
        {
            echo(" Package dropped, resetting gripper");

            ros::NodeHandle nh_ = cmd.nh;
            ros::Rate loopRate(10);  
            ros::Rate sleepRate(1);
            // ros::Publisher gripper_pub_ = nh_.advertise<std_msgs::Bool>("servo", 1);
            
            // std_msgs::UInt16 angle_msg;
            // echo("  Setting servo angle");
            // angle_msg.data = eq_angle;

            // gripper_pub_.publish(angle_msg);

            // echo("  Servo angle set to " << angle_msg.data);
            
            PkgAttached = false;
            return;
        }

        // location check functions

        bool isAtLZ(ros::NodeHandle nh)
        {
            ros::Subscriber mav_pose_sub_ = nh.subscribe("odometry", 10, mav_pose_cb_);
            ros::Rate loopRate(10);  
            ros::Rate sleepRate(1);

            double curr_pos[3], target_pos[3];
            bool AtLoc = false;

            echo("  Enroute to LZ, please wait");
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
                
                AtLoc = (dist > E) ? false : true;
                loopRate.sleep();
            }

            AtLZ = AtLoc;
            return AtLoc;
        }

        bool isAtMB(ros::NodeHandle nh)
        {
            ros::Subscriber mav_pose_sub_ = nh.subscribe("odometry", 10, mav_pose_cb_);
            ros::Rate loopRate(10);  
            ros::Rate sleepRate(1);

            double curr_pos[3], target_pos[3];
            bool AtLoc = false;

            echo("   Enroute to Mailbox, please wait");
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
                
                AtLoc = (dist > E) ? false : true;
                loopRate.sleep();
            }
            
            AtMailbox = AtLoc;
            return AtLoc;
        }

        // transition guard functions

        bool ReachedLZ(CmdHover const & cmd)
        {
            if(isAtLZ(cmd.nh))
            {
                echo(" Reached LZ");
            }

            return AtLZ;
        }

        bool ReachedMailbox(CmdHover const & cmd)
        {
            if(isAtMB(cmd.nh))
            {
                echo(" Reached Mailbox");
            }

            return AtMailbox;
        }

        template<class Event>
        bool NoPkg(Event const &){
            if(PkgAttached)
            {
                echo(" Pkg is attached, cannot proceed");
                return false;
            }
            else
            {
                echo(" Pkg is not attached, proceeding");
                return true;
            }
        }

        template<class Event>
        bool HasPkg(Event const &){
            if(PkgAttached)
            {
                echo(" Pkg is attached, proceeding");
                return true;
            }
            else
            {
                echo(" Pkg is not attached, cannot proceed");
                return false;
            }
        }

        bool ExecMission(CmdExploring const &){
            if(ContMission)
            {
                echo(" Executing Mission");
                return true;
            }
            else
            {
                echo(" Not executing Mission");
                return false;
            }
        }      
        
        template<class Event>
        bool StopMission(Event const &){
            if(!ContMission)
            {
                echo(" Not executing Mission");
                return true;
            }
            else
            {
                echo(" Executing Mission");
                return false;
            }
        }

        // transition table

        struct transition_table : mpl::vector<
        
        //      Type        Start            Event            Next              Action				    Guard
        // +++ ------- + -------------- + ------------- + -------------- + ------------------ + ---------------------- +++
                a_row<    Rest          ,  CmdTakeOff   ,  Hover         ,  &fsm::TakeOff                               >,
        // +++ ------- + -------------- + ------------- + -------------- + ------------------ + ---------------------- +++
                  row<    Hover         ,  CmdExploring ,  Exploring     ,  &fsm::Explore     ,  &fsm::ExecMission      >,
                  row<    Hover         ,  CmdGotoDrop  ,  ReachMailbox  ,  &fsm::GotoDrop    ,  &fsm::HasPkg           >,
                a_row<    Hover         ,  CmdDescent   ,  Descent       ,  &fsm::Descending                            >,
                  row<    Hover         ,  CmdGotoLZ    ,  ReachLZ       ,  &fsm::GotoLZ      ,  &fsm::StopMission      >,
        // +++ ------- + -------------- + ------------- + -------------- + ------------------ + ---------------------- +++
                a_row<    Exploring      ,  CmdHover     ,  Hover        ,  &fsm::Hovering                              >,
        // +++ ------- + -------------- + ------------- + -------------- + ------------------ + ---------------------- +++
                  row<    ReachMailbox  ,  CmdHover     ,  Hover         ,  &fsm::Hovering    ,  &fsm::ReachedMailbox   >,
        // +++ ------- + -------------- + ------------- + -------------- + ------------------ + ---------------------- +++
                  row<    Descent       ,  CmdDrop      ,  Drop          ,  &fsm::Dropping    ,  &fsm::HasPkg           >,
                  row<    Descent       ,  CmdAscent    ,  Hover         ,  &fsm::Ascending   ,  &fsm::NoPkg            >,
                  row<    Descent       ,  CmdLand      ,  Rest          ,  &fsm::Landing     ,  &fsm::StopMission      >,
        // +++ ------- + -------------- + ------------- + -------------- + ------------------ + ---------------------- +++
                  row<    Drop          ,  CmdDropOver  ,  Descent       ,  &fsm::DropOver    ,  &fsm::NoPkg            >,
        // +++ ------- + -------------- + ------------- + -------------- + ------------------ + ---------------------- +++
                  row<    ReachLZ       ,  CmdHover     ,  Hover         ,  &fsm::Hovering    ,  &fsm::ReachedLZ        >
        // +++ ------- + -------------- + ------------- + -------------- + ------------------ + ---------------------- +++
        
        >{};

    };

    // state machine back end declaration
    typedef msm::back::state_machine<fsm> fsm_;

    // state list
    static char const *const state_names[] = {"Rest",
                                            "Hover",
                                            "Exploring",
                                            "ReachMailbox",
                                            "Descent",
                                            "Drop",
                                            "ReachLZ"};

    // helper function -- output current state
    void curr_state(fsm_ const& msg)
    {
        echo("Current state -- " << state_names[msg.current_state()[0]]);
    }

    void statePublish(ros::NodeHandle nh, fsm_ *fsm)
    {
        ros::Publisher statePub = nh.advertise<std_msgs::String>("curr_state", 10);
        ros::Rate pubRate(10);
        std_msgs::String msg;
        while(ros::ok())
        {
            msg.data = state_names[fsm->current_state()[0]];
            statePub.publish(msg);
            pubRate.sleep();
        }
    }

}
