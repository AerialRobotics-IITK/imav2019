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
    mav_odom = msg;
}

namespace state_machine 
{

    namespace msm = boost::msm;
    namespace mpl = boost::mpl;

    struct CmdTakeOff{};
    struct CmdTrajectory{};
    struct CmdGetPkg{};
    struct CmdDescent{};
    struct CmdGotoDrop{};
    struct CmdAscent{};
    struct CmdDrop{};
    struct CmdLand{};
    struct CmdDropOver{};
    struct CmdHover{};

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
        bool PkgAttached = false;
        bool ExploreDone = false;
        bool AtLZ = false;
        bool AtMailbox = false;

        // void start_printing(print const &) { std::cout << "start_printet" << std::endl; }
        void TakeOff(CmdTakeOff const &) { std::cout << "Taking off" << std::endl; }
        void TrajExec(CmdTrajectory const &) { std::cout << "Exploring" << std::endl; ExploreDone=true;}
        void GetPkg(CmdGetPkg const &) { std::cout << "Going to LZ" << std::endl; AtLZ=true;}
        void GotoDrop(CmdGotoDrop const &) { std::cout << "Going to mailbox" << std::endl; AtMailbox=true;}
        void Ascending(CmdAscent const &) { std::cout << "Ascending" << std::endl; }
        void Dropping(CmdDrop const &) { std::cout << "Dropping" << std::endl; PkgAttached = false;}
        void Hovering(CmdHover const &) { std::cout << "Hovering" << std::endl; }
        void Landing(CmdLand const &) { std::cout << "Landing" << std::endl; PkgAttached = true;}
        // void Resting(CmdRest const &) { std::cout << "Resting" << std::endl; }
        void Descending(CmdDescent const &) { std::cout << "Descending" << std::endl; }
        void DropOver(CmdDropOver const &) { std::cout << "Dropped Pkg" << std::endl; }
        

        bool ReachedLZ(CmdHover const &)
        {
            if(AtLZ){
                std::cout << "Reached LZ" << std::endl;
            }
            else{
                std::cout << "Not at LZ yet" << std::endl;
            }
            return AtLZ;
        }

        bool ReachedMailbox(CmdHover const &)
        {
            if(AtMailbox){
                std::cout << "Reached Mailbox" << std::endl;
            }
            else{
                std::cout << "Not at mailbox yet" << std::endl;
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