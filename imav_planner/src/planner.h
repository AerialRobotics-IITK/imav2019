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

const double hover_height = 4.0;
const double drop_height = 0.3;
const double land_height = 0.08;

uint open_angle = 40;
uint close_angle = 140;
uint eq_angle = 90;

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
    std::cout << drop_info_.position.x << std::endl;
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

    struct CmdExplored
    {
        ros::NodeHandle nh;
        CmdExplored(ros::NodeHandle nh_) : nh(nh_) {}
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

    bool PkgAttached = false;
    bool AtLZ = false;
    bool AtMailbox = false;
    bool ContMission = true;

    struct fsm :  public msm::front::state_machine_def <fsm>
    {
        
        template <class Event, class FSM>
        void on_entry(Event const&, FSM& )
        {
            std::cout<< "Entered state machine"<<std::endl;
        }

        template <class Event, class FSM>
        void on_exit(Event const &, FSM &)
        {
            std::cout << "Exited state machine" << std::endl;
        }

        struct Rest : public msm::front::state <>
        {
            template <class Event, class FSM>
            void on_entry(Event const &, FSM &)
            {
                std::cout << "Entered Rest state" << std::endl;
            }

            template <class Event, class FSM>
            void on_exit(Event const &, FSM &)
            {
                std::cout << "Exited Rest state" << std::endl;
            }
        };

        struct Hover : public msm::front::state<>
        {
            template <class Event, class FSM>
            void on_entry(Event const &, FSM &)
            {
                std::cout << "Entered Hover mode" << std::endl;
            }

            template <class Event, class FSM>
            void on_exit(Event const &, FSM &)
            {
                std::cout << "Exited Hover mode" << std::endl;
            }
        };

        struct Explored : public msm::front::state<>
        {
            template <class Event, class FSM>
            void on_entry(Event const &, FSM &)
            {
                std::cout << "Entered Explored state" << std::endl;
            }

            template <class Event, class FSM>
            void on_exit(Event const &, FSM &)
            {
                std::cout << "Exited Explored state" << std::endl;
            }
        };

        struct ReachLZ : public msm::front::state<>
        {
            template <class Event, class FSM>
            void on_entry(Event const &, FSM &)
            {
                std::cout << "Entered ReachingLZ state" << std::endl;
            }

            template <class Event, class FSM>
            void on_exit(Event const &, FSM &)
            {
                std::cout << "Exited ReachingLZ state" << std::endl;
            }
        };

        struct ReachMailbox : public msm::front::state<>
        {
            template <class Event, class FSM>
            void on_entry(Event const &, FSM &)
            {
                std::cout << "Entered Reaching_Mailbox state" << std::endl;
            }

            template <class Event, class FSM>
            void on_exit(Event const &, FSM &)
            {
                std::cout << "Exited Reaching_Mailbox state" << std::endl;
            }
        };

        struct Descent : public msm::front::state<>
        {
            template <class Event, class FSM>
            void on_entry(Event const &, FSM &)
            {
                std::cout << "Entered Descent" << std::endl;
            }

            template <class Event, class FSM>
            void on_exit(Event const &, FSM &)
            {
                std::cout << "Exited Descent" << std::endl;
            } 
        };

        struct Drop : public msm::front::state<>
        {
            template <class Event, class FSM>
            void on_entry(Event const &, FSM &)
            {
                std::cout << "Entered Drop state" << std::endl;
            }

            template <class Event, class FSM>
            void on_exit(Event const &, FSM &)
            {
                std::cout << "Exited Drop state" << std::endl;
            }
        };

        typedef Rest initial_state;

        void TakeOff(CmdTakeOff const & cmd)
        {
            ros::NodeHandle nh_ = cmd.nh;
            ros::Rate loopRate(10);
            ros::ServiceClient takeoff_client_ = nh_.serviceClient<std_srvs::Empty>("takeoff");
            ros::Subscriber odom_sub_ = nh_.subscribe("ground_truth/odometry", 1, mav_pose_cb_);
            ros::Publisher gripper_pub_ = nh_.advertise<std_msgs::UInt16>("servo", 1);
            ros::Publisher command_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("command/pose", 1);

            std_msgs::UInt16 angle_msg;
            // when package is loaded angle is 140
            angle_msg.data = close_angle;
            int t=0;
            while(t++<10)
            {
                gripper_pub_.publish(angle_msg);
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

            std::cout << "Ascending" << std::endl;

            double curr_x, curr_y, curr_z=0;
            bool AscentDone = false;

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
            
            t=0;
            while(!AscentDone)
            {
                if(t++<10){
                    command_pub_.publish(command_msg);
                }
                ros::spinOnce();
                curr_z = mav_pose_.pose.pose.position.z;
                AscentDone = (curr_z < hover_height) ? false : true;
                loopRate.sleep();
            }

            PkgAttached = true;
            return;
        }

        void Exploring(CmdExplored const & cmd) 
        {
            std::cout << "Exploring" << std::endl;

            ros::NodeHandle nh_ = cmd.nh;
            // ros::Subscriber state_sub_ = nh_.subscribe("pilot/state", 100, state_cb_);
            ros::Subscriber mav_pose_sub_ = nh_.subscribe("ground_truth/odometry", 100, mav_pose_cb_);
            // ros::ServiceClient set_mode_client = nh_.serviceClient<mavros_msgs::SetMode>("pilot/set_mode");
            ros::Subscriber drop_sub_ = nh_.subscribe("drop_info", 100, drop_info_cb_);

            mavros_msgs::SetMode mission_set_mode, offb_set_mode;
            offb_set_mode.request.custom_mode = "OFFBOARD";
            mission_set_mode.request.custom_mode = "AUTO.MISSION";
            bool mode_set_= false;

            // while (!mode_set_)
            // {
            //     ros::spinOnce();
            //     if(mode_=="OFFBOARD")
            //     {
            //         if (set_mode_client.call(mission_set_mode) && mission_set_mode.response.mode_sent)
            //         {
            //             std::cout << "  Mission 1 enabled"<<std::endl;
            //             mode_set_=true;
            //         }
            //     }
            // }

            drop_info_.loc_type = "";
            while (drop_info_.loc_type != "Drop")
            {
                // int t = 0;
                // if(!(t++%10000))std::cout << "waiting for drop" << std::endl;
                ros::spinOnce();
                if(mav_pose_.pose.pose.position.x < 0.5 && mav_pose_.pose.pose.position.y < 0.5)
                {
                    if(!PkgAttached)
                    {
                        // while (mode_set_)
                        // {
                        //     ros::spinOnce();
                        //     if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                        //     {
                        //         std::cout << "  Offboard 1 enabled" << std::endl;
                        //         mode_set_ = false;
                        //     }
                        // }

                        ContMission = false;
                        return;
                    }
                }
            }

            // while (mode_set_)
            // {
            //     ros::spinOnce();
            //     if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            //     {
            //         std::cout << "  Offboard 2 enabled" << std::endl;
            //         mode_set_ = false;
            //     }
            // }

            return;
        }

        void GotoDrop(CmdGotoDrop const & cmd)
        {
            std::cout << "Going to Mailbox" << std::endl;

            ros::NodeHandle nh_ = cmd.nh;
            ros::Subscriber drop_info_sub_ = nh_.subscribe("drop_info", 1000, drop_info_cb_);
            ros::Publisher command_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("command/pose", 1000);
            
            geometry_msgs::PoseStamped command_msg;
            drop_info_.loc_type = "";
            while(drop_info_.loc_type != "Drop")
            {
                ros::spinOnce();
            }

            command_msg.header = drop_info_.header;
            command_msg.pose.position.x = drop_info_.position.x;
            command_msg.pose.position.y = drop_info_.position.y;
            command_msg.pose.position.z = hover_height;
            
            int t = 0;
            // without this loop the topic doesn't seem to catch the command message
            while(t++<1000)
            {
                command_pub_.publish(command_msg);
            }
            
            AtMailbox = true;
            return;
        }

        void Descending(CmdDescent const & cmd)
        {
            std::cout << "Descending" << std::endl;

            ros::NodeHandle nh_ = cmd.nh;
            ros::Rate loopRate(10);
            ros::Subscriber mav_pose_sub_ = nh_.subscribe("ground_truth/odometry", 1, mav_pose_cb_);
            ros::Publisher command_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("command/pose", 1);

            double curr_x, curr_y, curr_z=0;
            bool DescentDone = false;
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
            command_msg.pose.position.z = drop_height;

            int t=0;
            while(!DescentDone)
            {
                if(t++<10){
                    command_pub_.publish(command_msg);
                }

                ros::spinOnce();
                curr_z = mav_pose_.pose.pose.position.z;
                DescentDone = (curr_z > drop_height) ? false : true;
                loopRate.sleep();
            }

            return;      
        }

        void GotoLZ(CmdGotoLZ const & cmd)
        {
            std::cout << "Going to LZ" << std::endl;

            ros::NodeHandle nh_ = cmd.nh;
            ros::Subscriber home_info_sub_ = nh_.subscribe("home_info", 1000, home_info_cb_);
            ros::Publisher command_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("command/pose", 1000);

            geometry_msgs::PoseStamped command_msg;
            home_info_.loc_type = "";
            while(home_info_.loc_type != "Home")
            {
                ros::spinOnce();
            }

            command_msg.header = home_info_.header;
            command_msg.pose.position.x = home_info_.position.x;
            command_msg.pose.position.y = home_info_.position.y;
            command_msg.pose.position.z = hover_height;

            int t = 0; 
            // without this loop the topic doesn't seem to catch the command message
            while(t++<1000)
            {
                command_pub_.publish(command_msg);
            }
            
            AtLZ = true;
            return;
        }

        void Hovering(CmdHover const & cmd)
        {
            std::cout << "Hovering" << std::endl;

            ros::Rate loopRate(0.2);
            loopRate.sleep();            
        }

        void Dropping(CmdDrop const & cmd)
        {
            std::cout << "Dropping Package" << std::endl;

            ros::NodeHandle nh_ = cmd.nh;
            ros::Publisher gripper_pub_ = nh_.advertise<std_msgs::UInt16>("servo", 1);
            
            std_msgs::UInt16 angle_msg;
            angle_msg.data = open_angle;
            int t=0;
            while(t++<10)
            {
                gripper_pub_.publish(angle_msg);
            }
            std::cout << "Servo angle set to " << angle_msg.data << std::endl;
            
            PkgAttached = false;
            return;
        }

        void Ascending(CmdAscent const & cmd)
        {
            std::cout << "Ascending" << std::endl;

            ros::NodeHandle nh_ = cmd.nh;            
            ros::Rate loopRate(10);
            ros::Subscriber mav_pose_sub_ = nh_.subscribe("ground_truth/odometry", 1, mav_pose_cb_);
            ros::Publisher command_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("command/pose", 1);
            
            double curr_x, curr_y, curr_z=0;
            bool AscentDone = false;

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
                    command_pub_.publish(command_msg);
                }
                ros::spinOnce();
                curr_z = mav_pose_.pose.pose.position.z;
                AscentDone = (curr_z < hover_height) ? false : true;
                loopRate.sleep();
            }

            return;
        }

        void Landing(CmdLand const & cmd)
        {
            std::cout << "Landing" << std::endl;

            ros::NodeHandle nh_ = cmd.nh;
            ros::Rate loopRate(10);
            ros::Subscriber mav_pose_sub_ = nh_.subscribe("ground_truth/odometry", 1, mav_pose_cb_);
            ros::Publisher command_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("command/pose", 1);
            
            double curr_x, curr_y, curr_z=0;
            bool LandingDone = false;
            
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
                    command_pub_.publish(command_msg);
                }
                ros::spinOnce();
                curr_z = mav_pose_.pose.pose.position.z;
                LandingDone = (curr_z > land_height) ? false : true;
                loopRate.sleep();
            }

            return;
        }

        void DropOver(CmdDropOver const & cmd)
        {
            std::cout << "Package Dropped" << std::endl;

            ros::NodeHandle nh_ = cmd.nh;
            ros::Publisher gripper_pub_ = nh_.advertise<std_msgs::UInt16>("servo", 1);
            
            std_msgs::UInt16 angle_msg;
            angle_msg.data = eq_angle;
            int t=0;
            while(t++<10)
            {
                gripper_pub_.publish(angle_msg);
            }
            std::cout << "Servo angle set to " << angle_msg.data << std::endl;
            
            PkgAttached = false;
            return;
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

        bool isAtLZ(ros::NodeHandle nh)
        {
            ros::Subscriber mav_pose_sub_ = nh.subscribe("ground_truth/odometry", 1, mav_pose_cb_);
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
            ros::Subscriber mav_pose_sub_ = nh.subscribe("ground_truth/odometry", 1, mav_pose_cb_);
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

        bool ExecMission(CmdExplored const &){
            if(ContMission)
            {
                std::cout << "Executing Mission" << std::endl;
                return true;
            }
            else
            {
                std::cout << "Not executing Mission" << std::endl;
                return false;
            }
        }      
        
        template<class Event>
        bool StopMission(Event const &){
            if(!ContMission)
            {
                std::cout << "Not executing Mission" << std::endl;
                return true;
            }
            else
            {
                std::cout << "Executing Mission" << std::endl;
                return false;
            }
        }

        struct transition_table : mpl::vector<
                                    //    Start     Event         Next      Action				 Guard
                                    //  +---------+-------------+---------+---------------------+----------------------+
                                    a_row<Rest, CmdTakeOff, Hover, &fsm::TakeOff>,
                                    //  +---------+-------------+---------+---------------------+----------------------+
                                    row<Hover, CmdExplored, Explored, &fsm::Exploring, &fsm::ExecMission>,
                                    row<Hover, CmdGotoDrop, ReachMailbox, &fsm::GotoDrop, &fsm::HasPkg>,
                                    a_row<Hover, CmdDescent, Descent, &fsm::Descending>,
                                    row<Hover, CmdGotoLZ, ReachLZ, &fsm::GotoLZ, &fsm::StopMission>,
                                    //  +---------+-------------+---------+---------------------+----------------------+
                                    a_row<Explored, CmdHover, Hover, &fsm::Hovering>,
                                    //  +---------+-------------+---------+---------------------+----------------------+
                                    row<ReachMailbox, CmdHover, Hover, &fsm::Hovering, &fsm::ReachedMailbox>,
                                    //  +---------+-------------+---------+---------------------+----------------------+
                                    row<Descent, CmdDrop, Drop, &fsm::Dropping, &fsm::HasPkg>,
                                    row<Descent, CmdAscent, Hover, &fsm::Ascending, &fsm::NoPkg>,
                                    row<Descent, CmdLand, Rest, &fsm::Landing, &fsm::StopMission>,
                                    //  +---------+-------------+---------+---------------------+----------------------+
                                    row<Drop, CmdDropOver, Descent, &fsm::DropOver, &fsm::NoPkg>,
                                    //  +---------+-------------+---------+---------------------+----------------------+
                                    row<ReachLZ, CmdHover, Hover, &fsm::Hovering, &fsm::ReachedLZ>
                                    >
        {
        };

    };

    typedef msm::back::state_machine<fsm> fsm_;

    static char const *const state_names[] = {"Rest",
                                            "Hover",
                                            "Explored",
                                            "ReachMailbox",
                                            "Descent",
                                            "Drop",
                                            "ReachLZ"};

    void curr_state(fsm_ const& msg)
    {
        std::cout<<"Current state -- "<<state_names[msg.current_state()[0]]<<std::endl;
    }    
}