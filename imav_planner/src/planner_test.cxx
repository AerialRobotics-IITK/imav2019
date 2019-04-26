#include<ros/ros.h>
#include<nav_msgs/Odometry.h>
#include<geometry_msgs/PoseStamped.h>
#include<std_msgs/Bool.h>

#include <boost/msm/back/state_machine.hpp>
#include <boost/msm/back/mpl_graph_fsm_check.hpp>
#include <boost/msm/front/state_machine_def.hpp>
#include <boost/msm/front/functor_row.hpp>
#include <boost/msm/front/euml/common.hpp>
#include <boost/msm/front/euml/operator.hpp>

nav_msgs::Odometry mav_odom;

void Magnus_Callback(const nav_msgs::Odometry& msg)
{
    mav_odom=msg;
}

namespace msm = boost::msm;
namespace mpl = boost::mpl;

namespace{
struct CmdTakeOff
{
};
struct CmdTrajectory
{
};
struct CmdPickPlan
{
};
struct CmdDescent
{
};
struct CmdGrip
{
};
struct CmdDropPlan
{
};
struct CmdDrop
{
};
struct CmdLand
{
};
struct CmdRest
{
};
struct CmdHover
{
};

struct test_fsm : public msm::front::state_machine_def <test_fsm>
{

   

    template <class Event, class FSM>
    void on_entry(Event const&, FSM& )
    {
        std::cout<<"entering_fsm"<<std::endl;
    }

    template <class Event, class FSM>
    void on_exit(Event const &, FSM &)
    {
        std::cout << "Exiting_fsm" << std::endl;
    }

    // struct Empty : public msm::front::state <>
    // {
    //     template <class Event, class FSM>
    //     void on_entry(Event const &, FSM &)
    //     {
    //         std::cout << "entering_empty" << std::endl;
    //     }

    //     template <class Event, class FSM>
    //     void on_exit(Event const &, FSM &)
    //     {
    //         std::cout << "Exiting_empty" << std::endl;
    //     }
    // };

    struct Rest : public msm::front::state <>
    {
        template <class Event, class FSM>
        void on_entry(Event const &, FSM &)
        {
            std::cout << "entering_rest" << std::endl;
        }

        template <class Event, class FSM>
        void on_exit(Event const &, FSM &)
        {
            std::cout << "Exiting_rest" << std::endl;
        }
    };

    struct Hover : public msm::front::state<>
    {
        template <class Event, class FSM>
        void on_entry(Event const &, FSM &)
        {
            std::cout << "entering_hover" << std::endl;
        }

        template <class Event, class FSM>
        void on_exit(Event const &, FSM &)
        {
            std::cout << "Exiting_hover" << std::endl;
        }
    };

    struct Explore : public msm::front::state<>
    {
        template <class Event, class FSM>
        void on_entry(Event const &, FSM &)
        {
            std::cout << "entering_Explore" << std::endl;
        }

        template <class Event, class FSM>
        void on_exit(Event const &, FSM &)
        {
            std::cout << "Exiting_Explore" << std::endl;
        }
    };

    struct ReachingSetPt : public msm::front::state<>
    {
        template <class Event, class FSM>
        void on_entry(Event const &, FSM &)
        {
            std::cout << "entering_ReachingSetPt" << std::endl;
        }

        template <class Event, class FSM>
        void on_exit(Event const &, FSM &)
        {
            std::cout << "Exiting_ReachingSetPt" << std::endl;
        }
    };

    // struct ReachedSetPt : public msm::front::state<>
    // {
    //     template <class Event, class FSM>
    //     void on_entry(Event const &, FSM &)
    //     {
    //         std::cout << "entering_ReachedSetPt" << std::endl;
    //     }

    //     template <class Event, class FSM>
    //     void on_exit(Event const &, FSM &)
    //     {
    //         std::cout << "Exiting_ReachedSetPt" << std::endl;
    //     }
    // };

    struct Descent : public msm::front::state<>
    {
        template <class Event, class FSM>
        void on_entry(Event const &, FSM &)
        {
            std::cout << "entering_Descent" << std::endl;
        }

        template <class Event, class FSM>
        void on_exit(Event const &, FSM &)
        {
            std::cout << "Exiting_Descent" << std::endl;
        }
    };

    struct Grip : public msm::front::state<>
    {
        template <class Event, class FSM>
        void on_entry(Event const &, FSM &)
        {
            std::cout << "entering_Grip" << std::endl;
        }

        template <class Event, class FSM>
        void on_exit(Event const &, FSM &)
        {
            std::cout << "Exiting_Grip" << std::endl;
        }
    };

    struct Drop : public msm::front::state<>
    {
        template <class Event, class FSM>
        void on_entry(Event const &, FSM &)
        {
            std::cout << "entering_Drop" << std::endl;
        }

        template <class Event, class FSM>
        void on_exit(Event const &, FSM &)
        {
            std::cout << "Exiting_Drop" << std::endl;
        }
    };

    struct Land : public msm::front::state<>
    {
        template <class Event, class FSM>
        void on_entry(Event const &, FSM &)
        {
            std::cout << "entering_Land" << std::endl;
        }

        template <class Event, class FSM>
        void on_exit(Event const &, FSM &)
        {
            std::cout << "Exiting_Land" << std::endl;
        }
    };


    



    typedef Rest initial_state;

    // void start_printing(print const &) { std::cout << "start_printet" << std::endl; }
    void TakeOff(CmdTakeOff const &) { std::cout << "Taking_off" << std::endl; }
    void TrajExec(CmdTrajectory const &) { std::cout << "Exploring" << std::endl; }
    void GotoPickPt(CmdPickPlan const &) { std::cout << "Goto_pick" << std::endl; }
    void GotoDropPt(CmdDropPlan const &) { std::cout << "Goto_drop" << std::endl; }
    void Gripping(CmdGrip const &) { std::cout << "Gripping" << std::endl; }
    void Dropping(CmdDrop const &) { std::cout << "Dropping" << std::endl; }
    void Hovering(CmdHover const &) { std::cout << "Hovering" << std::endl; }
    void Landing(CmdLand const &) { std::cout << "Landing" << std::endl; }
    void Resting(CmdRest const &) { std::cout << "Resting" << std::endl; }
    void Descend(CmdDescent const &) { std::cout << "Descending" << std::endl; }
    
    bool ReachedSetPt(CmdHover const &)
    {
        std::cout<<"NotReachedSetPt"<<std::endl;
        return false;
    } 

    typedef test_fsm tfsm;

    struct transition_table : mpl::vector<
                                  //    Start     Event         Next      Action				 Guard
                                  //  +---------+-------------+---------+---------------------+----------------------+
                                  a_row<Rest, CmdTakeOff, Hover, &tfsm::TakeOff>,
                                  //  +---------+-------------+---------+---------------------+----------------------+
                                  a_row<Hover, CmdTrajectory, Explore, &tfsm::TrajExec>,
                                  a_row<Hover, CmdPickPlan, ReachingSetPt, &tfsm::GotoPickPt>,
                                  a_row<Hover, CmdDescent, Descent, &tfsm::Descend>,
                                  //  +---------+-------------+---------+---------------------+----------------------+
                                  a_row<Descent, CmdGrip, Grip, &tfsm::Gripping>,
                                  a_row<Descent, CmdDrop, Drop, &tfsm::Dropping>,
                                  //  +---------+-------------+---------+---------------------+----------------------+
                                  a_row<Grip, CmdDropPlan, ReachingSetPt, &tfsm::GotoDropPt>,
                                  //  +---------+-------------+---------+---------------------+----------------------+
                                  a_row<Drop, CmdHover, Hover, &tfsm::Hovering>,
                                  a_row<Drop, CmdLand, Land, &tfsm::Landing>,
                                  //  +---------+-------------+---------+---------------------+----------------------+
                                  a_row<Land, CmdRest, Rest, &tfsm::Resting>,
                                  //  +---------+-------------+---------+---------------------+----------------------+
                                  row<ReachingSetPt, CmdHover, Hover, &tfsm::Hovering, &tfsm::ReachedSetPt>,
                                  //  +---------+-------------+---------+---------------------+----------------------+
                                  a_row<Explore, CmdHover, Hover, &tfsm::Hovering>
                                  >
    {
    };
};


typedef msm::back::state_machine<test_fsm> test_fsm_;

static char const *const state_names[] = {"Rest",
                                          "Hover",
                                          "Descent",
                                          "Grip",
                                          "Drop",
                                          "Land",
                                          "ReachingSetPt",
                                          "Explore"};

void machine_state(test_fsm_ const& msg)
{
    std::cout<<"current_state -- "<<state_names[msg.current_state()[0]]<<std::endl;
}

void test()
{
    test_fsm_ machine;
    machine.start();
    machine.process_event(CmdTakeOff()); machine_state(machine);
    machine.process_event(CmdTrajectory()); machine_state(machine);
    machine.process_event(CmdHover()); machine_state(machine);
    machine.process_event(CmdPickPlan()); machine_state(machine);
    machine.process_event(CmdHover()); machine_state(machine);
    machine.process_event(CmdDescent()); machine_state(machine);
    machine.process_event(CmdGrip()); machine_state(machine);
    machine.process_event(CmdDropPlan()); machine_state(machine);
    machine.process_event(CmdHover()); machine_state(machine);
    machine.process_event(CmdDescent()); machine_state(machine);
    machine.process_event(CmdDrop()); machine_state(machine);
    machine.process_event(CmdLand()); machine_state(machine);
    machine.process_event(CmdRest()); machine_state(machine);
    std::cout<<"stop_fsm"<<std::endl;
    machine.stop();
}
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"planner_test");
    ros::NodeHandle nh;

    ros::Subscriber magnus_subscriber = nh.subscribe("ground_truth/odometry",10,Magnus_Callback);

    ros::Publisher magnus_publisher = nh.advertise<geometry_msgs::PoseStamped>("command/pose", 1);
    ros::Publisher gripper_pub = nh.advertise<std_msgs::Bool>("gripper_status", 1);
    test();
    ros::Rate loopRate(10);
    

    while(ros::ok())
    {
        geometry_msgs::PoseStamped mav_cmd_pose_;
        std_msgs::Bool gripper_status_;

        magnus_publisher.publish(mav_cmd_pose_);
        ros::spinOnce();
        loopRate.sleep();
    }
    
    return 0;
}