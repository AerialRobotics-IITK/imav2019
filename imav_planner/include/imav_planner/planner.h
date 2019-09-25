#include <ros/ros.h>
#include <future>
#include <chrono>

#include <GeographicLib/UTMUPS.hpp>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>

#include <std_srvs/Empty.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/String.h>

#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/HomePosition.h>
#include <mavros_msgs/WaypointPull.h>
#include <mavros_msgs/WaypointReached.h>
#include <mavros_msgs/GripperServo.h>
#include <mavros_msgs/State.h>

#include <mav_utils_msgs/TaskInfo.h>
#include <mav_utils_msgs/signal.h>
#include <mav_utils_msgs/BBPoses.h>
#include <mav_utils_msgs/UTMPose.h>

#include <boost/msm/back/state_machine.hpp>
#include <boost/msm/back/mpl_graph_fsm_check.hpp>
#include <boost/msm/front/state_machine_def.hpp>
#include <boost/msm/front/functor_row.hpp>
#include <boost/msm/front/euml/common.hpp>
#include <boost/msm/front/euml/operator.hpp>

#define echo(X) std::cout << X << std::endl
#define sq(X) (X)*(X)

// storage variables
nav_msgs::Odometry mav_pose_, return_pose_;
mav_utils_msgs::UTMPose utm_pose_, home_pose_;
mav_utils_msgs::BBPoses obj_data, helipad;
mav_utils_msgs::TaskInfo drop_info_;
// geometry_msgs::PointStamped home_info_;
mavros_msgs::WaypointReached prev_wp;
mavros_msgs::State mav_mode_;
geometry_msgs::PointStamped home_msg_;

// error for location comparison
double loc_error = 0.05;

// verbose flag
bool verbose = true;

// height params - tuned for tfmini
double hover_height = 4.0;
double drop_height = 1.2;
double land_height = 0.41;
double descent_step = 0.4;

// servo angles
double open_angle = 40;
double close_angle = 140;
double eq_angle = 90;

// Rates
double hover_time = 5.0;
double transition_time = 5.0;
double wait_time = 20.0;
double exit_time = 20.0;
double land_wait = 10.0;

// callback functions

void mav_pose_cb_(const nav_msgs::Odometry &msg){mav_pose_ = msg;}
// void home_info_cb_(const geometry_msgs::PointStamped &msg){home_info_ = msg;}
void drop_info_cb_(const mav_utils_msgs::TaskInfo &msg){drop_info_ = msg;}
void obj_cb_(const mav_utils_msgs::BBPoses &msg){obj_data = msg;}
void helipad_cb_(const mav_utils_msgs::BBPoses &msg){helipad = msg;}
void utm_pose_cb_(const mav_utils_msgs::UTMPose &msg){utm_pose_ = msg;}
void wp_reached_cb_(const mavros_msgs::WaypointReached &msg){prev_wp = msg;}
void state_cb_(const mavros_msgs::State &msg){mav_mode_ = msg;}

// state machine definition

namespace state_machine{

    namespace msm = boost::msm;
    namespace mpl = boost::mpl;

    // state machine commands

    struct CmdTakeOff{CmdTakeOff(){}};
    struct CmdExploring{CmdExploring(){}};
    struct CmdGotoLZ{CmdGotoLZ(){}};
    struct CmdDescent{CmdDescent(){}};
    struct CmdGotoDrop{CmdGotoDrop(){}};
    struct CmdAscent{CmdAscent(){}};
    struct CmdDrop{CmdDrop(){}};
    struct CmdLand{CmdLand(){}};
    struct CmdDropOver{CmdDropOver(){}};
    struct CmdHover{CmdHover(){}};

    // state variables

    bool PkgAttached = true;
    bool AtLZ = false;
    bool AtMailbox = false;
    bool ContMission = true;
    bool HoverMode = false;
    int LastDrop = -INT_MAX;

    // state machine object

    struct fsm :  public msm::front::state_machine_def <fsm>{

        typedef msm::active_state_switch_before_transition active_state_switch_policy;

        template <class Event, class FSM> void on_entry(Event const&, FSM& ){if(verbose)   echo("Entered state machine");}
        template <class Event, class FSM> void on_exit(Event const &, FSM &){if(verbose)   echo("Exited state machine");}

        // state definitions

        struct Rest : public msm::front::state <>{
            template <class Event, class FSM> void on_entry(Event const &, FSM &){if(verbose)   echo("Entered Rest state");}
            template <class Event, class FSM> void on_exit(Event const &, FSM &){if(verbose)   echo("Exited Rest state");}
        };
        struct Hover : public msm::front::state<>{
            template <class Event, class FSM> void on_entry(Event const &, FSM &){if(verbose)   echo("Entered Hover mode");}
            template <class Event, class FSM> void on_exit(Event const &, FSM &){if(verbose)   echo("Exited Hover mode");}
        };
        struct Exploring : public msm::front::state<>
        {
            template <class Event, class FSM> void on_entry(Event const &, FSM &){if(verbose)   echo("Entered Exploring state");}
            template <class Event, class FSM> void on_exit(Event const &, FSM &){if(verbose)   echo("Exited Exploring state");}
        };
        struct ReachLZ : public msm::front::state<>{
            template <class Event, class FSM> void on_entry(Event const &, FSM &){if(verbose)   echo("Entered ReachingLZ state");}
            template <class Event, class FSM> void on_exit(Event const &, FSM &){if(verbose)   echo("Exited ReachingLZ state");}
        };
        struct ReachMailbox : public msm::front::state<>{
            template <class Event, class FSM> void on_entry(Event const &, FSM &){if(verbose)   echo("Entered Reaching_Mailbox state");}
            template <class Event, class FSM> void on_exit(Event const &, FSM &){if(verbose)   echo("Exited Reaching_Mailbox state");}
        };
        struct Descent : public msm::front::state<>{
            template <class Event, class FSM> void on_entry(Event const &, FSM &){if(verbose)   echo("Entered Descent");}
            template <class Event, class FSM> void on_exit(Event const &, FSM &){if(verbose)   echo("Exited Descent");} 
        };
        struct Drop : public msm::front::state<>{
            template <class Event, class FSM> void on_entry(Event const &, FSM &){if(verbose)   echo("Entered Drop state");}
            template <class Event, class FSM> void on_exit(Event const &, FSM &){if(verbose)   echo("Exited Drop state");}
        };

        // initial state declaration
        typedef Rest initial_state;

        // global node handle
        ros::NodeHandle nh;

        // publishers
        ros::Publisher command_pub_ = nh.advertise<geometry_msgs::PointStamped>("mission_info", 10);
        ros::Publisher gripper_pub_ = nh.advertise<mavros_msgs::GripperServo>("servo", 1);
        ros::Publisher pose_pub_ = nh.advertise<mav_utils_msgs::BBPoses>("object_poses", 1);

        // subscribers
        ros::Subscriber drop_info_sub_ = nh.subscribe("drop_info", 1, drop_info_cb_);
        // ros::Subscriber home_info_sub_ = nh.subscribe("home_info", 1, home_info_cb_);
        ros::Subscriber utm_pose_sub_ = nh.subscribe("utm_pose", 1, utm_pose_cb_);
        ros::Subscriber mav_pose_sub_ = nh.subscribe("odometry", 10, mav_pose_cb_);
        ros::Subscriber mission_wp_sub = nh.subscribe("mission/reached", 10, wp_reached_cb_);
        ros::Subscriber state_sub_ = nh.subscribe("state",1, state_cb_);
        ros::Subscriber obj_sub_ = nh.subscribe("object_poses", 10, obj_cb_);
        ros::Subscriber heli_sub_ = nh.subscribe("helipad", 10, helipad_cb_);    

        //  service clients
        ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("set_mode");
        ros::ServiceClient mission_client = nh.serviceClient<mavros_msgs::WaypointPull>("mission/pull");
        ros::ServiceClient detector_client = nh.serviceClient<mav_utils_msgs::signal>("detector/terminate");
        ros::ServiceClient helipad_client = nh.serviceClient<mav_utils_msgs::signal>("hdetect/terminate");
        // ros::ServiceClient pose_hold_client = nh.serviceClient<std_srvs::Empty>("back_to_position_hold");

        // state transition functions

        void TakeOff(CmdTakeOff const & cmd){
            if(verbose)   echo(" Starting execution");

            ros::Rate loopRate(10);

            if(verbose)   echo("   Publishing servo command");
            mavros_msgs::GripperServo gripper_msg;
            gripper_msg.frame_stamp = ros::Time::now();
            gripper_msg.servo_setpoint = close_angle;
            gripper_pub_.publish(gripper_msg);
            if(verbose)   echo("   Published servo command");
            
            if(verbose)   echo("  Servo angle set to " << (int) gripper_msg.servo_setpoint);

            PkgAttached = true;

            utm_pose_.pose.position.z = -DBL_MAX;
            if(verbose)   echo("  Waiting for UTM position");
            while(utm_pose_.pose.position.z == -DBL_MAX){
                ros::spinOnce();
                loopRate.sleep();
            }
            if(verbose)   echo("  Received UTM position");
            home_pose_ = utm_pose_;

            return;
        }

        void Explore(CmdExploring const & cmd){
            if(verbose)   echo(" Exploring");
            ros::Rate loopRate(10);

            geometry_msgs::PointStamped mission_msg;

            mav_utils_msgs::signal start;
            start.request.signal = 1;
            if(detector_client.call(start) && start.response.success){
                if(verbose)   echo("  Started detector node");
            }

            mavros_msgs::WaypointPull req;
            int num_wp=0;
            while(num_wp == 0)
            {
                if(mission_client.call(req) && req.response.success){
                    num_wp = req.response.wp_received - 1;
                }
            }
            if(verbose)     echo("  Received waypoints");

            if(verbose)   echo("  Waiting for odometry");
            mav_pose_.pose.pose.position.z = -DBL_MAX;
            while(mav_pose_.pose.pose.position.z == -DBL_MAX){
                ros::spinOnce();
                loopRate.sleep();
            }
            if(verbose)   echo("  Received odometry");

            mavros_msgs::SetMode mission_set_mode, offb_set_mode;
            offb_set_mode.request.custom_mode = "OFFBOARD";
            mission_set_mode.request.custom_mode = "AUTO.MISSION";
            bool mode_set_= false;

            std_srvs::Empty pose_hold;

            if(verbose)   echo("  Changing mode to Mission");                                                           // Add state check
            while (!mode_set_){
                ros::spinOnce();
                if (set_mode_client.call(mission_set_mode) && mission_set_mode.response.mode_sent){
                    if(verbose)   echo("   Mission enabled");
                    mode_set_=true;
                }
                loopRate.sleep();
            }
            if(verbose)   echo("  Changed mode to Mission");

            drop_info_.loc_type = "";
            if(verbose)   echo("  Searching for mailboxes");
            double home_dist = 0;
            while(((drop_info_.loc_type != "Drop" || !PkgAttached) && drop_info_.loc_type != "Hover") || drop_info_.id == LastDrop){
                ros::spinOnce();

                // exit condition
                if(prev_wp.wp_seq == num_wp){
                    if(verbose)     echo("  Reached last waypoint, validating home");
                    // PkgAttached = false;    // remove this, it should happen automatically

                    if(!PkgAttached){
                        if(verbose)   echo("   No package attached, switching from Mission mode");
                        while (mode_set_){
                            mission_msg.header.stamp = ros::Time::now();
                            mission_msg.point = mav_pose_.pose.pose.position;

                            command_pub_.publish(mission_msg);

                            if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) mode_set_ = false;
                            loopRate.sleep();
                        }
                        if(verbose)   echo("   Switched to Offboard");

                        ContMission = HoverMode = false;
                        return;
                    }
                    else; // what happens when drops are not done when mission ends?
                }
                loopRate.sleep();
            }

            if(verbose)   echo("  Mailbox detected, switching from Mission mode");
            if(verbose)   echo("   Drop location: x = " << drop_info_.position.x << ", y = " << drop_info_.position.y);

            // drop_info_sub_.shutdown();
  
            // if(verbose)    echo("   Drop info subscriber shutdown");
            // if(verbose)    echo("   Drop location: x = " << drop_info_.position.x << ", y = " << drop_info_.position.y);

            while (mode_set_){
                ros::spinOnce();

                mission_msg.header.stamp = ros::Time::now();
                mission_msg.point = mav_pose_.pose.pose.position;
                
                command_pub_.publish(mission_msg);
                
                while(mav_mode_.mode == "AUTO.MISSION"){
                    if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) mode_set_ = false;
                    ros::spinOnce();
                }
                if(verbose)   echo("   Offboard enabled");
                return_pose_ = mav_pose_;

                loopRate.sleep();
            }
            if(verbose)   echo("  Switched to Offboard");

            if(drop_info_.loc_type == "Hover") HoverMode = true;
            else HoverMode = false;
            LastDrop = drop_info_.id;

            return;
        }

        void GotoDrop(CmdGotoDrop const & cmd){
            if(verbose)   echo(" Going to Mailbox");
            ros::Rate loopRate(10);

            geometry_msgs::PointStamped mission_msg;

            if(verbose)   echo("  Checking for drop location");
            if(drop_info_.loc_type != "Drop" && drop_info_.loc_type != "Hover"){ 
                if(verbose)   echo("  Waiting for drop location");
                while(drop_info_.loc_type != "Drop" && drop_info_.loc_type != "Hover"){
                    ros::spinOnce();
                    loopRate.sleep();
                }
            }
            if(verbose)   echo("  Received drop location");
            
            mission_msg.header.stamp = ros::Time::now();
            mission_msg.point.x = drop_info_.position.x;
            mission_msg.point.y = drop_info_.position.y;
            mission_msg.point.z = hover_height;

            if(verbose)   echo("   Mission location: x = " << mission_msg.point.x << ", y = " << mission_msg.point.y << " type " << drop_info_.loc_type);
 
            command_pub_.publish(mission_msg);
            return;
        }

        void Descending(CmdDescent const & cmd){
            if(verbose)   echo(" Descending");
            ros::Rate loopRate(10);

            bool DescentDone = false;
            geometry_msgs::PointStamped mission_msg;
            obj_data.object_poses.clear();

            if(verbose)   echo("  Checking for drop location");
            if(drop_info_.loc_type != "Drop" && drop_info_.loc_type != "Hover"){ 
                if(verbose)   echo("  Waiting for drop location");
                while(drop_info_.loc_type != "Drop" && drop_info_.loc_type != "Hover"){
                    ros::spinOnce();
                    loopRate.sleep();
                }
            }
            if(verbose)   echo("  Received drop location");
            
            if(verbose)   echo("  Waiting for odometry");
            mav_pose_.pose.pose.position.z = -DBL_MAX;
            while(mav_pose_.pose.pose.position.z == -DBL_MAX){
                ros::spinOnce();
                loopRate.sleep();
            }
            if(verbose)   echo("  Received odometry");

            int i = 0;
            if(drop_info_.loc_type == "Drop"){
                if(verbose)   echo("  Starting descent");

                while(!DescentDone){
                    mission_msg.header.stamp = ros::Time::now();
                    if(obj_data.object_poses.size()>0){
                        for(i = 0; i < obj_data.object_poses.size(); i++){
                            if(obj_data.object_poses.at(i).type == drop_info_.id) break;
                        }
                        mission_msg.point.x = obj_data.object_poses.at(i).position.x;
                        mission_msg.point.y = obj_data.object_poses.at(i).position.y;
                    }
                    else{
                        mission_msg.point.x = mav_pose_.pose.pose.position.x;
                        mission_msg.point.y = mav_pose_.pose.pose.position.y;
                    }
                    mission_msg.point.z = mav_pose_.pose.pose.position.z - descent_step;
                    command_pub_.publish(mission_msg);
                    DescentDone = (mav_pose_.pose.pose.position.z > drop_height) ? false : true;
                    ros::spinOnce();
                    loopRate.sleep();
                }

            }
            else if(drop_info_.loc_type == "Hover"){
                if(verbose)   echo("  Starting hover over target");
                std::chrono::steady_clock::time_point stop = std::chrono::steady_clock::now() + std::chrono::seconds((int) exit_time);
                mav_utils_msgs::BBPose data; int imageID;
                while(!DescentDone){
                    mission_msg.header.stamp = ros::Time::now();
                    if(obj_data.object_poses.size()>0){
                        for(i = 0; i < obj_data.object_poses.size(); i++){
                            if(obj_data.object_poses.at(i).type == drop_info_.id) break;
                        }
                        data = obj_data.object_poses.at(i);
                        imageID = obj_data.imageID;
                        mission_msg.point.x = obj_data.object_poses.at(i).position.x;
                        mission_msg.point.y = obj_data.object_poses.at(i).position.y;
                    }
                    else{
                        mission_msg.point.x = mav_pose_.pose.pose.position.x;
                        mission_msg.point.y = mav_pose_.pose.pose.position.y;
                    }
                    mission_msg.point.z = mav_pose_.pose.pose.position.z;
                    command_pub_.publish(mission_msg);
                    if(std::chrono::steady_clock::now() > stop){
                        if(verbose)     echo("Time's up!");
                        obj_data.object_poses.clear();
                        data.store = true; DescentDone = true;
                        obj_data.object_poses.push_back(data);
                        obj_data.imageID = imageID;
                        obj_data.stamp = ros::Time::now();
                        pose_pub_.publish(obj_data);
                        break;
                    }
                    else if(obj_data.object_poses.size()>0){
                        for(int j=0; j < obj_data.object_poses.size(); j++){
                            if(obj_data.object_poses.at(j).type == drop_info_.id){
                                DescentDone = obj_data.object_poses.at(j).store;
                                break;
                            }
                        }
                    }
                    else DescentDone = false;
                    ros::spinOnce();
                    loopRate.sleep();
                }
            }
            if(verbose)   echo("  Descent done");
            return;      
        }

        void GotoLZ(CmdGotoLZ const & cmd){
            if(verbose)   echo(" Going to LZ");
            ros::Rate loopRate(10);

            geometry_msgs::PointStamped mission_msg;

            mav_utils_msgs::signal stop;
            stop.request.signal = 0;
            if(detector_client.call(stop) && stop.response.success){
                if(verbose)   echo("  detector node stopped");
            }
            
            // home_info_.point.z = -DBL_MAX;
            // if(verbose)   echo("  Waiting for Home location");
            // while(home_info_.point.z == -DBL_MAX){
            //     ros::spinOnce();
            //     loopRate.sleep();
            // }
            // if(verbose)   echo("  Home location received");

            utm_pose_.pose.position.z = -DBL_MAX;
            if(verbose)   echo("  Waiting for UTM position");
            while(utm_pose_.pose.position.z == -DBL_MAX){
                ros::spinOnce();
                loopRate.sleep();
            }
            if(verbose)   echo("  Received UTM position");

            mav_pose_.pose.pose.position.z = -DBL_MAX;
            if(verbose)   echo("  Waiting for odometry");
            while(mav_pose_.pose.pose.position.z == -DBL_MAX){
                ros::spinOnce();
                loopRate.sleep();
            }

            mission_msg.header.stamp = ros::Time::now();
            mission_msg.point.x = home_pose_.pose.position.x + mav_pose_.pose.pose.position.x - utm_pose_.pose.position.x;
            mission_msg.point.y = home_pose_.pose.position.y + mav_pose_.pose.pose.position.y - utm_pose_.pose.position.y;
            mission_msg.point.z = hover_height;

            if(verbose)   echo("  Home location: x = " << mission_msg.point.x << ", y = " << mission_msg.point.y);
            home_msg_ = mission_msg;

            command_pub_.publish(mission_msg);
            return;
        }

        void Hovering(CmdHover const & cmd){
            if(verbose)   echo(" Hovering");
            ros::Rate hoverRate(1.0/hover_time);
            hoverRate.sleep();            
        }

        void Dropping(CmdDrop const & cmd){
            if(HoverMode){
                if(verbose)     echo("Hover mode, passing through");
                return;
            }
            if(verbose)   echo(" Dropping Package");

            mavros_msgs::GripperServo gripper_msg;
            gripper_msg.frame_stamp = ros::Time::now();
            gripper_msg.servo_setpoint = open_angle;
            gripper_pub_.publish(gripper_msg);
            
            if(verbose)   echo("  Servo angle set to " << (int) gripper_msg.servo_setpoint);
            
            PkgAttached = false;
            return;
        }

        void Ascending(CmdAscent const & cmd){
            ros::Rate loopRate(10);

            double dist = 0;
            bool ReturnDone = false;
            bool AscentDone = false;
            geometry_msgs::PointStamped mission_msg;

            if(verbose)   echo("  Waiting for odometry");
            mav_pose_.pose.pose.position.z = -DBL_MAX;
            while(mav_pose_.pose.pose.position.z == -DBL_MAX){
                ros::spinOnce();
                loopRate.sleep();
            }
            if(verbose)   echo("  Received odometry");

            if(HoverMode){
                if(verbose)   echo("  Hover mode, passing through ascent");
            }
            else{
                mission_msg.header.stamp = ros::Time::now();
                mission_msg.point.x = mav_pose_.pose.pose.position.x;
                mission_msg.point.y = mav_pose_.pose.pose.position.y;
                mission_msg.point.z = hover_height;

                if(verbose)   echo("  Starting ascent");

                command_pub_.publish(mission_msg);

                while(!AscentDone){
                    ros::spinOnce();
                    AscentDone = (mav_pose_.pose.pose.position.z < hover_height) ? false : true;
                    loopRate.sleep();
                }
                if(verbose)   echo("  Ascent done");
            }

            mission_msg.header.stamp = ros::Time::now();
            mission_msg.point.x = return_pose_.pose.pose.position.x;
            mission_msg.point.y = return_pose_.pose.pose.position.y;
            mission_msg.point.z = hover_height;

            if(verbose)   echo("  Returning to mission at: x = " << mission_msg.point.x << ", y = " << mission_msg.point.y);
            if(verbose)   echo("  Starting return to mission");
            
            command_pub_.publish(mission_msg);
            while(!ReturnDone){
                ros::spinOnce();
                dist = sq(mav_pose_.pose.pose.position.x - return_pose_.pose.pose.position.x) + sq(mav_pose_.pose.pose.position.y - return_pose_.pose.pose.position.y);
                ReturnDone = (dist > sq(loc_error)) ? false : true;
                loopRate.sleep();
            }
            if(verbose) echo("  Return done");

            return;
        }

        void Landing(CmdLand const & cmd){
            if(verbose)   echo(" Landing"); 
            ros::Rate loopRate(10);
            // rewrite

            mavros_msgs::SetMode land_set_mode;
            bool mode_set_ = false;
            land_set_mode.request.custom_mode = "AUTO.LAND";

            mav_utils_msgs::signal start;
            start.request.signal = 1;
            if(helipad_client.call(start) && start.response.success){
                if(verbose)   echo("  hdetect node started");
            }

            bool LandingDone = false;
            geometry_msgs::PointStamped mission_msg;

            if(verbose)   echo("  Waiting for odometry");
            mav_pose_.pose.pose.position.z = -DBL_MAX;
            while(mav_pose_.pose.pose.position.z == -DBL_MAX){
                ros::spinOnce();
                loopRate.sleep();
            }
            if(verbose)   echo("  Received odometry");

            if(verbose)   echo("  Waiting for helipad position");
            helipad.imageID = -INT32_MAX;
            std::chrono::steady_clock::time_point stop = std::chrono::steady_clock::now() + std::chrono::seconds((int) exit_time);
            mav_utils_msgs::BBPose data; int imageID;
            while(helipad.imageID == -INT32_MAX){
                ros::spinOnce();
                if(helipad.object_poses.size() > 0) data = helipad.object_poses.at(0);
                imageID = helipad.imageID;
                if(std::chrono::steady_clock::now() > stop){
                    if(verbose)     echo("Time's up!");
                    helipad.object_poses.clear();
                    helipad.object_poses.push_back(data);
                    helipad.imageID = imageID;
                    break;
                }
                loopRate.sleep();
            }
            if(verbose)   echo("  Received helipad position");

            if(verbose)   echo("  Landing initiated");
            int i = 0;
            while(!LandingDone){
                mission_msg.header.stamp = ros::Time::now();
                if(helipad.object_poses.size()>0){
                    mission_msg.point.x = helipad.object_poses.at(0).position.x;
                    mission_msg.point.y = helipad.object_poses.at(0).position.y;
                }
		        else{
                    mission_msg.point.x = mav_pose_.pose.pose.position.x;
                    mission_msg.point.y = mav_pose_.pose.pose.position.y;
                }
                mission_msg.point.z = mav_pose_.pose.pose.position.z - descent_step;
                command_pub_.publish(mission_msg);
                LandingDone = (mav_pose_.pose.pose.position.z > land_height) ? false : true;
                ros::spinOnce();
                loopRate.sleep();
            }
            
            while (!mode_set_)
            {
                ros::spinOnce();
                if (set_mode_client.call(land_set_mode) && land_set_mode.response.mode_sent)
                {
                    if (verbose)
                        echo("   Land enabled");
                    mode_set_ = true;
                }
                loopRate.sleep();
            }

            if(verbose) echo("  Changed mode to Land");
            if(verbose) echo("  Landing done");

            return;
        }

        void DropOver(CmdDropOver const & cmd){
            if(HoverMode){
                if(verbose)     echo("Hover mode, passing through");
                return;
            }
            if(verbose)   echo(" Package dropped, resetting gripper");
            
            // std_msgs::UInt16 angle_msg;
            // if(verbose)    echo("  Setting servo angle");
            // angle_msg.data = eq_angle;

            // gripper_pub_.publish(angle_msg);

            // if(verbose)    echo("  Servo angle set to " << angle_msg.data);
            return;
        }

        // location check functions

        bool isAtLZ(ros::NodeHandle nh){
            double dist = 0;
            bool AtLoc = false;
            ros::Rate loopRate(10);

            // home_info_.point.z = -DBL_MAX;
            // if(verbose)   echo("  Waiting for home location");
            // while(home_info_.point.z == -DBL_MAX){
            //     ros::spinOnce();
            //     loopRate.sleep();
            // }
            // if(verbose)   echo("  Received home location");

            if(verbose)   echo("  Waiting for odometry");
            mav_pose_.pose.pose.position.z = -DBL_MAX;
            while(mav_pose_.pose.pose.position.z == -DBL_MAX){
                ros::spinOnce();
                loopRate.sleep();
            }
            if(verbose)   echo("  Received odometry");

            if(verbose)   echo("  Enroute to LZ, please wait");
            while(!AtLoc){
                ros::spinOnce();
                dist = sq(mav_pose_.pose.pose.position.x - home_msg_.point.x) + sq(mav_pose_.pose.pose.position.y - home_msg_.point.y);
                AtLoc = (dist > sq(loc_error)) ? false : true;
                loopRate.sleep();
            }

            AtLZ = AtLoc;
            return AtLoc;
        }

        bool isAtMB(ros::NodeHandle nh){
            double dist = 0;
            bool AtLoc = false;
            ros::Rate loopRate(10);

            if(verbose)   echo("  Checking for drop info");
            if(drop_info_.loc_type != "Drop" && drop_info_.loc_type != "Hover"){
                if(verbose)   echo("  Waiting for drop location");
                while(drop_info_.loc_type != "Drop" && drop_info_.loc_type != "Hover"){
                    ros::spinOnce();
                    loopRate.sleep();
                }
            }
            if(verbose)   echo("  Received Drop location");

            if(verbose)   echo("  Waiting for odometry");
            mav_pose_.pose.pose.position.z = -DBL_MAX;
            while(mav_pose_.pose.pose.position.z == -DBL_MAX){
                ros::spinOnce();
                loopRate.sleep();
            }
            if(verbose)   echo("  Received odometry");       

            if(verbose)   echo("   Enroute to Mailbox, please wait");
            while(!AtLoc){
                ros::spinOnce();
                dist = sq(mav_pose_.pose.pose.position.x - drop_info_.position.x) + sq(mav_pose_.pose.pose.position.y - drop_info_.position.y);
                AtLoc = (dist > sq(loc_error)) ? false : true;
                loopRate.sleep();
            }

            AtMailbox = AtLoc;
            return AtLoc;
        }

        // transition guard functions

        bool ReachedLZ(CmdHover const & cmd){
            if(isAtLZ(nh)){if(verbose)   echo(" Reached LZ");}
            return AtLZ;
        }

        bool ReachedMailbox(CmdHover const & cmd){
            if(isAtMB(nh)){if(verbose)   echo(" Reached Mailbox");}
            return AtMailbox;
        }

        template<class Event> bool NoPkg(Event const &){
            if(HoverMode) return true;

            if(PkgAttached){
                if(verbose)   echo(" Pkg is attached, cannot proceed");
                return false;
            }
            else{
                if(verbose)   echo(" Pkg is not attached, proceeding");
                return true;
            }
        }

        template<class Event> bool HasPkg(Event const &){
            if(HoverMode) return true;

            if(PkgAttached){
                if(verbose)   echo(" Pkg is attached, proceeding");
                return true;
            }
            else{
                if(verbose)   echo(" Pkg is not attached, cannot proceed");
                return false;
            }
        }

        bool ExecMission(CmdExploring const &){
            if(ContMission){
                if(verbose)   echo(" Executing Mission");
                return true;
            }
            else{
                if(verbose)   echo(" Not executing Mission");
                return false;
            }
        }      
        
        template<class Event> bool StopMission(Event const &){
            if(!ContMission){
                if(verbose)   echo(" Not executing Mission");
                return true;
            }
            else{
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
                a_row<    Exploring     ,  CmdHover     ,  Hover         ,  &fsm::Hovering                              >,
        // +++ ------- + -------------- + ------------- + -------------- + ------------------ + ---------------------- +++
                  row<    ReachMailbox  ,  CmdHover     ,  Hover         ,  &fsm::Hovering    ,  &fsm::ReachedMailbox   >,
        // +++ ------- + -------------- + ------------- + -------------- + ------------------ + ---------------------- +++
                  row<    Descent       ,  CmdDrop      ,  Drop          ,  &fsm::Dropping    ,  &fsm::HasPkg           >,
                  row<    Descent       ,  CmdAscent    ,  Hover         ,  &fsm::Ascending   ,  &fsm::NoPkg            >,
                  row<    Hover         ,  CmdLand      ,  Rest          ,  &fsm::Landing     ,  &fsm::StopMission      >,
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
    void echo_state(fsm_ const& msg){if(verbose)   echo("Current state -- " << state_names[msg.current_state()[0]]);}

    // state publisher
    void statePublish(ros::NodeHandle nh, fsm_ *fsm)
    {
        ros::Publisher statePub = nh.advertise<std_msgs::String>("curr_state", 10);
        ros::Rate loopRate(10);

        std_msgs::String msg;
        while(ros::ok()){
            msg.data = state_names[fsm->current_state()[0]];
            statePub.publish(msg);
            loopRate.sleep();
        }
    }
}
