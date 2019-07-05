#include <ros/ros.h>
#include <future>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/NavSatFix.h>

#include <std_srvs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/HomePosition.h>
#include <mavros_msgs/State.h>

#include <mav_utils_msgs/TaskInfo.h>
#include <mav_utils_msgs/signal.h>
#include <mav_utils_msgs/BBPoses.h>
#include <mav_utils_msgs/MissionInfo.h>

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

#define echo(X) std::cout << X << std::endl
#define sq(X) X*X

// storage variables
nav_msgs::Odometry mav_pose_;
sensor_msgs::NavSatFix global_pose_;
mav_utils_msgs::BBPoses obj_data, helipad;
mav_utils_msgs::TaskInfo drop_info_;
mavros_msgs::HomePosition home_info_;
std_msgs::Bool gripper_status_;

// errors for location comparison
double gps_error = 1.0;
double loc_error = 1.0;

// verbose flag
bool verbose = true;

// height params - tuned for tfmini
double hover_height = 4.0;
double drop_height = 1.2;
double land_height = 0.41;
double descent_step = 0.04;

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

void home_info_cb_(const mavros_msgs::HomePosition &msg)
{
    home_info_ = msg;
}

void drop_info_cb_(const mav_utils_msgs::TaskInfo &msg)
{
    drop_info_ = msg;
    // echo("Drop location: x = " << drop_info_.position.x << ", y = " << drop_info_.position.y);
}

void obj_cb_(const mav_utils_msgs::BBPoses &msg)
{
    obj_data = msg;
}

void helipad_cb_(const mav_utils_msgs::BBPoses &msg)
{
    helipad = msg;
}

void global_pose_cb_(const sensor_msgs::NavSatFix &msg)
{
    global_pose_ = msg;
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
    int numPackages = 0;

    // rates
    ros::Rate loopRate(10);  
    ros::Rate sleepRate(1);
    ros::Rate hoverRate(1.0/hover_time);

    // state machine object

    struct fsm :  public msm::front::state_machine_def <fsm>
    {
        typedef msm::active_state_switch_before_transition active_state_switch_policy;

        template <class Event, class FSM>
        void on_entry(Event const&, FSM& )
        {
            if(verbose)   echo("Entered state machine");
        }

        template <class Event, class FSM>
        void on_exit(Event const &, FSM &)
        {
            if(verbose)   echo("Exited state machine");
        }

        // state definitions

        struct Rest : public msm::front::state <>
        {
            template <class Event, class FSM>
            void on_entry(Event const &, FSM &)
            {
                if(verbose)   echo("Entered Rest state");
            }

            template <class Event, class FSM>
            void on_exit(Event const &, FSM &)
            {
                if(verbose)   echo("Exited Rest state");
            }
        };

        struct Hover : public msm::front::state<>
        {
            template <class Event, class FSM>
            void on_entry(Event const &, FSM &)
            {
                if(verbose)   echo("Entered Hover mode");
            }

            template <class Event, class FSM>
            void on_exit(Event const &, FSM &)
            {
                if(verbose)   echo("Exited Hover mode");
            }
        };

        struct Exploring : public msm::front::state<>
        {
            template <class Event, class FSM>
            void on_entry(Event const &, FSM &)
            {
                if(verbose)   echo("Entered Exploring state");
            }

            template <class Event, class FSM>
            void on_exit(Event const &, FSM &)
            {
                if(verbose)   echo("Exited Exploring state");
            }
        };

        struct ReachLZ : public msm::front::state<>
        {
            template <class Event, class FSM>
            void on_entry(Event const &, FSM &)
            {
                if(verbose)   echo("Entered ReachingLZ state");
            }

            template <class Event, class FSM>
            void on_exit(Event const &, FSM &)
            {
                if(verbose)   echo("Exited ReachingLZ state");
            }
        };

        struct ReachMailbox : public msm::front::state<>
        {
            template <class Event, class FSM>
            void on_entry(Event const &, FSM &)
            {
                if(verbose)   echo("Entered Reaching_Mailbox state");
            }

            template <class Event, class FSM>
            void on_exit(Event const &, FSM &)
            {
                if(verbose)   echo("Exited Reaching_Mailbox state");
            }
        };

        struct Descent : public msm::front::state<>
        {
            template <class Event, class FSM>
            void on_entry(Event const &, FSM &)
            {
                if(verbose)   echo("Entered Descent");
            }

            template <class Event, class FSM>
            void on_exit(Event const &, FSM &)
            {
                if(verbose)   echo("Exited Descent");
            } 
        };

        struct Drop : public msm::front::state<>
        {
            template <class Event, class FSM>
            void on_entry(Event const &, FSM &)
            {
                if(verbose)   echo("Entered Drop state");
            }

            template <class Event, class FSM>
            void on_exit(Event const &, FSM &)
            {
                if(verbose)   echo("Exited Drop state");
            }
        };

        // initial state declaration
        typedef Rest initial_state;

        // state transition functions

        void TakeOff(CmdTakeOff const & cmd)
        {
            if(verbose)   echo(" Starting execution");
            ros::NodeHandle nh_ = cmd.nh;
            
            ros::Publisher gripper_pub_ = nh_.advertise<std_msgs::Bool>("servo", 1);

            // std_msgs::Bool angle_msg;
            // if(verbose)    echo("  Setting servo angle");
            // angle_msg.data = close_angle;

            if(verbose)   echo("   Publishing servo command");
            std_msgs::Bool angle_msg;
            angle_msg.data = true;
            gripper_pub_.publish(angle_msg);
            if(verbose)   echo("   Published servo command");
            
            if(verbose)   echo("  Servo angle set to " << (int) angle_msg.data);

            PkgAttached = true;
            numPackages++;

            return;
        }

        void Explore(CmdExploring const & cmd) 
        {
            if(verbose)   echo(" Exploring");
            ros::NodeHandle nh_ = cmd.nh;

            ros::Subscriber drop_info_sub_ = nh_.subscribe("drop_info", 1, drop_info_cb_);
            ros::Subscriber home_info_sub_ = nh_.subscribe("home_info", 1, home_info_cb_);
            ros::Subscriber global_pose_sub_ = nh_.subscribe("global_pose", 1, global_pose_cb_);

            ros::Publisher command_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("command/pose", 10);

            ros::ServiceClient pose_hold_client = nh_.serviceClient<std_srvs::Empty>("back_to_position_hold");
            ros::ServiceClient set_mode_client = nh_.serviceClient<mavros_msgs::SetMode>("set_mode");
            ros::ServiceClient node_client = nh_.serviceClient<mav_utils_msgs::signal>("detector/terminate");

            mav_utils_msgs::signal start;
            start.request.signal = 1;
            if(node_client.call(start) && start.response.success)
            {
                if(verbose)   echo("  Started detector node");
            }

            home_info_.geo.latitude = 0;
            if(verbose)   echo("  Waiting for Home location");
            while(home_info_.geo.latitude == 0)
            {
                ros::spinOnce();
                loopRate.sleep();
            }
            if(verbose)   echo("  Home location received");

            global_pose_.latitude = 0;
            if(verbose)   echo("  Waiting for GPS position");
            while(global_pose_.latitude == 0)
            {
                ros::spinOnce();
                loopRate.sleep();
            }
            if(verbose)   echo("  Received GPS position");

            mavros_msgs::SetMode mission_set_mode, offb_set_mode;
            offb_set_mode.request.custom_mode = "OFFBOARD";
            mission_set_mode.request.custom_mode = "AUTO.MISSION";
            bool mode_set_= false;

            std_srvs::Empty pose_hold;

            if(verbose)   echo("  Changing mode to Mission");                                                           // Add state check
            while (!mode_set_)
            {
                ros::spinOnce();
                if (set_mode_client.call(mission_set_mode) && mission_set_mode.response.mode_sent)
                {
                    if(verbose)   echo("   Mission enabled");
                    mode_set_=true;
                }
                loopRate.sleep();
            }
            if(verbose)   echo("  Changed mode to Mission");

            drop_info_.loc_type = "";
            if(verbose)   echo("  Searching for mailboxes");
            while (drop_info_.loc_type != "Drop" || drop_info_.loc_type != "Hover")
            {
                ros::spinOnce();

                // exit condition
                if(sq(global_pose_.latitude - home_info_.geo.latitude) + sq(global_pose_.longitude - home_info_.geo.longitude) < gps_error)            // LZ positions
                {
                    if(verbose)   echo("   Reached LZ, mission over.");
				    PkgAttached=false;

                    if(!PkgAttached)
                    {
                        if(verbose)   echo("   No package attached, switching from Mission mode");
                        while (mode_set_)
                        {
                            if(pose_hold_client.call(pose_hold))
                            {
                                if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                                {
                                    if(verbose)   echo("    Offboard enabled");
                                    mode_set_ = false;
                                }
                            }
                            loopRate.sleep();
                        }
                        if(verbose)   echo("   Switched to Offboard");

                        ContMission = false;
                        return;
                    }
                }

                loopRate.sleep();
            }

            if(verbose)   echo("  Mailbox detected, switching from Mission mode");
            if(verbose)
            {
                if(!drop_info_.is_local) echo("   Drop location: lat = " << drop_info_.latitude << ", lon = " << drop_info_.longitude);
                else echo("   Drop location: x = " << drop_info_.position.x << ", lon = " << drop_info_.position.y);
            }

            drop_info_sub_.shutdown();
  
            // if(verbose)    echo("   Drop info subscriber shutdown");
            // if(verbose)    echo("   Drop location: x = " << drop_info_.position.x << ", y = " << drop_info_.position.y);

            while (mode_set_)
            {
                if(pose_hold_client.call(pose_hold))
                {
                    if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                    {
                        if(verbose)   echo("   Offboard enabled");
                        mode_set_ = false;
                    }
                }
                loopRate.sleep();
            }
            if(verbose)   echo("  Switched to Offboard");

            return;
        }

        void GotoDrop(CmdGotoDrop const & cmd)
        {
            if(verbose)   echo(" Going to Mailbox");
            ros::NodeHandle nh_ = cmd.nh;

            ros::Subscriber drop_info_sub_ = nh_.subscribe("drop_info", 10, drop_info_cb_);
            
            ros::Publisher command_pub_ = nh_.advertise<mav_utils_msgs::MissionInfo>("mission_info", 10);

            mav_utils_msgs::MissionInfo mission_msg;
            if(verbose)   echo("  Checking for drop location");
            if(drop_info_.loc_type != "Drop" || drop_info_.loc_type != "Hover"){ 
                if(verbose)   echo("  Waiting for drop location");
                while(drop_info_.loc_type != "Drop" || drop_info_.loc_type != "Hover")
                {
                    ros::spinOnce();
                    loopRate.sleep();
                }
            }
            if(verbose)   echo("  Received drop location");
            
            mission_msg.header.stamp = ros::Time::now();
            
            if(drop_info_.is_local)
            {
                mission_msg.use_local = true;
                mission_msg.use_local = true;
                mission_msg.local_pose.x = drop_info_.position.x;
                mission_msg.local_pose.y = drop_info_.position.y;
                mission_msg.local_pose.z = hover_height;
                if(verbose)   echo("   Mission location: x = " << mission_msg.local_pose.x << ", y = " << mission_msg.local_pose.y << "type " << drop_info_.loc_type);
            }
            else
            {
                mission_msg.use_local = false;
                mission_msg.use_nmpc = false;
                mission_msg.altitude = hover_height;
                mission_msg.latitude = drop_info_.latitude;
                mission_msg.longitude = drop_info_.longitude;
                if(verbose)   echo("   Mission location: lat = " << mission_msg.latitude << ", lon = " << mission_msg.longitude << "type " << drop_info_.loc_type);
            }

            command_pub_.publish(mission_msg);

            return;
        }

        void Descending(CmdDescent const & cmd)
        {
            if(verbose)   echo(" Descending");
                        ros::NodeHandle nh_ = cmd.nh;
            
            ros::Subscriber mav_pose_sub_ = nh_.subscribe("odometry", 10, mav_pose_cb_);
            ros::Subscriber obj_sub_ = nh_.subscribe("object_poses", 10, obj_cb_);
            
            ros::Publisher command_pub_ = nh_.advertise<mav_utils_msgs::MissionInfo>("mission_info", 10);

            double curr_x, curr_y, curr_z=0;
            bool DescentDone = false;
            mav_utils_msgs::MissionInfo mission_msg;

            if(verbose)   echo("  Checking for drop location");
            if(drop_info_.loc_type != "Drop" || drop_info_.loc_type != "Hover"){ 
                if(verbose)   echo("  Waiting for drop location");
                while(drop_info_.loc_type != "Drop" || drop_info_.loc_type != "Hover")
                {
                    ros::spinOnce();
                    loopRate.sleep();
                }
            }
            if(verbose)   echo("  Received drop location");
            
            if(verbose)   echo("  Waiting for odometry");
            while(curr_z==0)
            {
                ros::spinOnce();
                curr_x = mav_pose_.pose.pose.position.x;
                curr_y = mav_pose_.pose.pose.position.y;
                curr_z = mav_pose_.pose.pose.position.z;
                loopRate.sleep();
            }
            if(verbose)   echo("  Received odometry");

            if(drop_info_.loc_type == "Drop")
            {
                if(verbose)   echo("  Starting descent");

                while(!DescentDone)
                {
                    mission_msg.header.stamp = ros::Time::now();
                    mission_msg.use_local = true;
                    mission_msg.use_nmpc = true;
                    if(obj_data.object_poses.size()>0)
                    {
                        mission_msg.local_pose.x = obj_data.object_poses.at(0).position.x;
                        mission_msg.local_pose.y = obj_data.object_poses.at(0).position.y;
                    }
                    else
                    {
                        mission_msg.local_pose.x = mav_pose_.pose.pose.position.x;
                        mission_msg.local_pose.y = mav_pose_.pose.pose.position.y;
                    }
                    mission_msg.local_pose.z = mav_pose_.pose.pose.position.z - descent_step;
                    command_pub_.publish(mission_msg);
                    DescentDone = (mav_pose_.pose.pose.position.z > drop_height) ? false : true;
                    ros::spinOnce();
                    loopRate.sleep();
                }
        
            }
            else if(drop_info_.loc_type == "Hover")
            {
                if(verbose)   echo("  Starting hover over target");
                while(!DescentDone)
                {
                    mission_msg.header.stamp = ros::Time::now();
                    mission_msg.use_local = true;
                    mission_msg.use_nmpc = true;
                    if(obj_data.object_poses.size()>0)
                    {
                        mission_msg.local_pose.x = obj_data.object_poses.at(0).position.x;
                        mission_msg.local_pose.y = obj_data.object_poses.at(0).position.y;
                    }
                    else
                    {
                        mission_msg.local_pose.x = mav_pose_.pose.pose.position.x;
                        mission_msg.local_pose.y = mav_pose_.pose.pose.position.y;
                    }
                    mission_msg.local_pose.z = mav_pose_.pose.pose.position.z;
                    command_pub_.publish(mission_msg);
                    DescentDone = (obj_data.object_poses.size()>0) ? obj_data.object_poses.at(0).store : false;
                    ros::spinOnce();
                    loopRate.sleep();
                }
            } 
            
            if(verbose)   echo("  Descent done");
            return;      
        }

        void GotoLZ(CmdGotoLZ const & cmd)
        {
            if(verbose)   echo(" Going to LZ");
            ros::NodeHandle nh_ = cmd.nh;
            
            ros::Subscriber home_info_sub_ = nh_.subscribe("home_info", 1, home_info_cb_);
            
            ros::Publisher command_pub_ = nh_.advertise<mav_utils_msgs::MissionInfo>("mission_info", 10);

            mav_utils_msgs::MissionInfo mission_msg;
            home_info_.header.seq = 0;
            if(verbose)   echo("  Waiting for Home location");
            while(home_info_.header.seq == 0)
            {
                ros::spinOnce();
                loopRate.sleep();
            }
            if(verbose)   echo("  Home location received");

            mission_msg.header.stamp = ros::Time::now();
            mission_msg.use_local = false;
            mission_msg.use_nmpc = false;
            mission_msg.latitude = home_info_.geo.latitude;
            mission_msg.longitude = home_info_.geo.longitude;
            mission_msg.altitude = hover_height;

            command_pub_.publish(mission_msg);
            return;
        }

        void Hovering(CmdHover const & cmd)
        {
            if(verbose)   echo(" Hovering");
            hoverRate.sleep();            
        }

        void Dropping(CmdDrop const & cmd)
        {
            if(verbose)   echo(" Dropping Package");
            ros::NodeHandle nh_ = cmd.nh;
            
            ros::Publisher gripper_pub_ = nh_.advertise<std_msgs::Bool>("servo", 1);
            
            // std_msgs::UInt16 angle_msg;
            // if(verbose)    echo("  Setting servo angle");
            // angle_msg.data = open_angle;

            std_msgs::Bool angle_msg;
            angle_msg.data = false;
            gripper_pub_.publish(angle_msg);
            
            if(verbose)   echo("  Servo angle set to " << angle_msg.data);
            
            numPackages--;
            PkgAttached = (numPackages > 0);
            return;
        }

        void Ascending(CmdAscent const & cmd)
        {
            if(verbose)   echo(" Ascending");
            ros::NodeHandle nh_ = cmd.nh;
            
            ros::Subscriber mav_pose_sub_ = nh_.subscribe("odometry", 10, mav_pose_cb_);
            
            ros::Publisher command_pub_ = nh_.advertise<mav_utils_msgs::MissionInfo>("mission_info", 10);
            
            double curr_x, curr_y, curr_z=0;
            bool AscentDone = false;

            mav_utils_msgs::MissionInfo mission_msg;
            if(verbose)   echo("  Waiting for odometry");
            while(curr_z==0)
            {
                ros::spinOnce();
                curr_x = mav_pose_.pose.pose.position.x;
                curr_y = mav_pose_.pose.pose.position.y;
                curr_z = mav_pose_.pose.pose.position.z;
                loopRate.sleep();
            }
            if(verbose)   echo("  Received odometry");

            mission_msg.header.stamp = ros::Time::now();
            mission_msg.use_local = true;
            mission_msg.use_nmpc = true;
            mission_msg.local_pose.x = mav_pose_.pose.pose.position.x;
            mission_msg.local_pose.y = mav_pose_.pose.pose.position.y;
            mission_msg.local_pose.z = hover_height;

            if(verbose)   echo("  Starting ascent");

            command_pub_.publish(mission_msg);
            
            while(!AscentDone)
            {
                ros::spinOnce();
                curr_z = mav_pose_.pose.pose.position.z;
                AscentDone = (curr_z < hover_height) ? false : true;
                loopRate.sleep();
            }
            if(verbose)   echo("  Ascent done");

            return;
        }

        void Landing(CmdLand const & cmd)
        {
            if(verbose)   echo(" Landing");
            ros::NodeHandle nh_ = cmd.nh;
            
            ros::Subscriber mav_pose_sub_ = nh_.subscribe("odometry", 10, mav_pose_cb_);
            ros::Subscriber heli_sub_ = nh_.subscribe("helipad", 10, helipad_cb_);
            
            ros::Publisher command_pub_ = nh_.advertise<mav_utils_msgs::MissionInfo>("mission_info", 10);
            
            ros::ServiceClient detector_client = nh_.serviceClient<mav_utils_msgs::signal>("detector/terminate");
            ros::ServiceClient helipad_client = nh_.serviceClient<mav_utils_msgs::signal>("hdetect/terminate");

            mav_utils_msgs::signal stop, start;
            stop.request.signal = 0;
            start.request.signal = 1;
            if(detector_client.call(stop) && stop.response.success)
            {
                if(verbose)   echo("  detector node stopped");
            }
            if(helipad_client.call(start) && start.response.success)
            {
                if(verbose)   echo("  hdetect node started");
            }

            double curr_x, curr_y, curr_z=0;
            bool LandingDone = false;
            
            mav_utils_msgs::MissionInfo mission_msg;
            if(verbose)   echo("  Waiting for odometry");
            while(curr_z==0)
            {
                ros::spinOnce();
                curr_x = mav_pose_.pose.pose.position.x;
                curr_y = mav_pose_.pose.pose.position.y;
                curr_z = mav_pose_.pose.pose.position.z;
                loopRate.sleep();
            }
            if(verbose)   echo("  Received odometry");

            if(verbose)   echo("  Waiting for helipad position");
            helipad.imageID = 0;
            while(helipad.imageID == 0)
            {
                ros::spinOnce();
                loopRate.sleep();
            }
            if(verbose)   echo("  Received helipad position");

            if(verbose)   echo("  Landing initiated");
            while(!LandingDone)
            {
                mission_msg.header.stamp = ros::Time::now();
                mission_msg.use_local = true;
                mission_msg.use_nmpc = true;
                if(helipad.object_poses.size()>0)
                {
                    mission_msg.local_pose.x = helipad.object_poses.at(0).position.x;
                    mission_msg.local_pose.y = helipad.object_poses.at(0).position.y;
                }
                else
                {
                    mission_msg.local_pose.x = mav_pose_.pose.pose.position.x;
                    mission_msg.local_pose.y = mav_pose_.pose.pose.position.y;
                }
                mission_msg.local_pose.z = mav_pose_.pose.pose.position.z - descent_step;
                command_pub_.publish(mission_msg);
                LandingDone = (mav_pose_.pose.pose.position.z > land_height) ? false : true;
                ros::spinOnce();
                loopRate.sleep();
            }
            if(verbose)   echo("  Landing done");

            return;
        }

        void DropOver(CmdDropOver const & cmd)
        {
            if(verbose)   echo(" Package dropped, resetting gripper");
            ros::NodeHandle nh_ = cmd.nh;

            // ros::Publisher gripper_pub_ = nh_.advertise<std_msgs::Bool>("servo", 1);
            
            // std_msgs::UInt16 angle_msg;
            // if(verbose)    echo("  Setting servo angle");
            // angle_msg.data = eq_angle;

            // gripper_pub_.publish(angle_msg);

            // if(verbose)    echo("  Servo angle set to " << angle_msg.data);
            
            return;
        }

        // location check functions

        bool isAtLZ(ros::NodeHandle nh)
        {
            ros::Subscriber home_info_sub_ = nh.subscribe("home_info", 10, home_info_cb_);
            ros::Subscriber global_pose_sub_ = nh.subscribe("global_pose", 10, global_pose_cb_);

            double dist = 0;
            bool AtLoc = false;

            home_info_.geo.latitude = 0;
            if(verbose)   echo("  Waiting for home location");
            while(home_info_.geo.latitude == 0)
            {
                ros::spinOnce();
                loopRate.sleep();
            }
            if(verbose)   echo("  Received home location");

            if(verbose)   echo("  Waiting for GPS position");
            global_pose_.latitude = 0;
            while(global_pose_.latitude == 0)
            {
                ros::spinOnce();
                loopRate.sleep();
            }
            if(verbose)   echo("  Received GPS postion");

            if(verbose)   echo("  Enroute to LZ, please wait");
            while(!AtLoc)
            {
                ros::spinOnce();
                dist = sq(global_pose_.latitude - home_info_.geo.latitude) + sq(global_pose_.longitude - home_info_.geo.longitude);
                AtLoc = (dist > gps_error) ? false : true;
                loopRate.sleep();
            }

            AtLZ = AtLoc;
            return AtLoc;
        }

        bool isAtMB(ros::NodeHandle nh)
        {
            ros::Subscriber mav_pose_sub_ = nh.subscribe("odometry", 10, mav_pose_cb_);
            ros::Subscriber drop_info_sub_ = nh.subscribe("drop_info", 10, drop_info_cb_);
            ros::Subscriber global_pose_sub_ = nh.subscribe("global_pose", 10, global_pose_cb_);

            double dist = 0;
            double curr_x, curr_y, curr_z = 0;
            bool AtLoc = false;

            if(verbose)   echo("  Checking for drop info");
            if(drop_info_.loc_type != "Drop" || drop_info_.loc_type != "Hover")
            {
                if(verbose)   echo("  Waiting for drop location");
                while(drop_info_.loc_type != "Drop" || drop_info_.loc_type != "Hover")
                {
                    ros::spinOnce();
                    loopRate.sleep();
                }
            }
            if(verbose)   echo("  Received Drop location");

            if(verbose)   echo("  Waiting for odometry");
            while(curr_z==0)
            {
                ros::spinOnce();
                curr_x = mav_pose_.pose.pose.position.x;
                curr_y = mav_pose_.pose.pose.position.y;
                curr_z = mav_pose_.pose.pose.position.z;
                loopRate.sleep();
            }
            if(verbose)   echo("  Received odometry");

            if(verbose)   echo("  Waiting for GPS position");
            global_pose_.latitude = 0;
            while(global_pose_.latitude == 0)
            {
                ros::spinOnce();
                loopRate.sleep();
            }
            if(verbose)   echo("  Received GPS postion");            

            if(verbose)   echo("   Enroute to Mailbox, please wait");
            if(drop_info_.is_local)
            {
                while(!AtLoc)
                {
                    ros::spinOnce();
                    dist = sq(mav_pose_.pose.pose.position.x - drop_info_.position.x) + sq(mav_pose_.pose.pose.position.y - drop_info_.position.y);
                    AtLoc = (dist > loc_error) ? false : true;
                    loopRate.sleep();
                }
            }
            else
            {
                while(!AtLoc)
                {
                    ros::spinOnce();
                    dist = sq(global_pose_.latitude - drop_info_.latitude) + sq(global_pose_.longitude - drop_info_.longitude);
                    AtLoc = (dist > gps_error) ? false : true;
                    loopRate.sleep();
                }
            }

            AtMailbox = AtLoc;
            return AtLoc;
        }

        // transition guard functions

        bool ReachedLZ(CmdHover const & cmd)
        {
            if(isAtLZ(cmd.nh))
            {
                if(verbose)   echo(" Reached LZ");
            }

            return AtLZ;
        }

        bool ReachedMailbox(CmdHover const & cmd)
        {
            if(isAtMB(cmd.nh))
            {
                if(verbose)   echo(" Reached Mailbox");
            }

            return AtMailbox;
        }

        template<class Event>
        bool NoPkg(Event const &){
            if(PkgAttached)
            {
                if(verbose)   echo(" Pkg is attached, cannot proceed");
                return false;
            }
            else
            {
                if(verbose)   echo(" Pkg is not attached, proceeding");
                return true;
            }
        }

        template<class Event>
        bool HasPkg(Event const &){
            if(PkgAttached)
            {
                if(verbose)   echo(" Pkg is attached, proceeding");
                return true;
            }
            else
            {
                if(verbose)   echo(" Pkg is not attached, cannot proceed");
                return false;
            }
        }

        bool ExecMission(CmdExploring const &){
            if(ContMission)
            {
                if(verbose)   echo(" Executing Mission");
                return true;
            }
            else
            {
                if(verbose)   echo(" Not executing Mission");
                return false;
            }
        }      
        
        template<class Event>
        bool StopMission(Event const &){
            if(!ContMission)
            {
                if(verbose)   echo(" Not executing Mission");
                return true;
            }
            else
            {
                if(verbose)   echo(" Executing Mission");
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
    void echo_state(fsm_ const& msg)
    {
        if(verbose)   echo("Current state -- " << state_names[msg.current_state()[0]]);
    }

    // state publisher
    void statePublish(ros::NodeHandle nh, fsm_ *fsm)
    {
        ros::Publisher statePub = nh.advertise<std_msgs::String>("curr_state", 10);
        
        std_msgs::String msg;
        while(ros::ok())
        {
            msg.data = state_names[fsm->current_state()[0]];
            statePub.publish(msg);
            loopRate.sleep();
        }
    }

}
