#include "planner_test.h"

int main(int argc, char **argv)
{
    ros::init(argc,argv,"planner_test");
    
    state_machine::test_fsm_ machine;
    machine.start();
    machine.process_event(state_machine::CmdTakeOff()); state_machine::curr_state(machine);
    machine.process_event(state_machine::CmdTrajectory()); state_machine::curr_state(machine);
    machine.process_event(state_machine::CmdHover()); state_machine::curr_state(machine);
    machine.process_event(state_machine::CmdGetPkg()); state_machine::curr_state(machine);
    machine.process_event(state_machine::CmdHover()); state_machine::curr_state(machine);
    machine.process_event(state_machine::CmdDescent()); state_machine::curr_state(machine);
    machine.process_event(state_machine::CmdLand()); state_machine::curr_state(machine);
    machine.process_event(state_machine::CmdTakeOff()); state_machine::curr_state(machine);
    machine.process_event(state_machine::CmdGotoDrop()); state_machine::curr_state(machine);
    machine.process_event(state_machine::CmdHover()); state_machine::curr_state(machine);
    machine.process_event(state_machine::CmdDescent()); state_machine::curr_state(machine);
    machine.process_event(state_machine::CmdDrop()); state_machine::curr_state(machine);
    machine.process_event(state_machine::CmdDropOver()); state_machine::curr_state(machine);
    machine.process_event(state_machine::CmdAscent()); state_machine::curr_state(machine);
    machine.process_event(state_machine::CmdGetPkg()); state_machine::curr_state(machine);
    machine.process_event(state_machine::CmdHover());state_machine::curr_state(machine);
    machine.process_event(state_machine::CmdDescent()); state_machine::curr_state(machine);
    machine.process_event(state_machine::CmdLand()); state_machine::curr_state(machine);
    std::cout<<"End Mission"<<std::endl;
    machine.stop();
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