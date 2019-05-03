#include "planner_test.h"

int main(int argc, char **argv)
{
    ros::init(argc,argv,"planner_test");

    ros::NodeHandle nh;

    ros::Subscriber mav_pose_sub = nh.subscribe("odometry", 1, mav_pose_cb_);

    ros::Subscriber gripper_status_sub = nh.subscribe("gripper_status", 1,gripper_status_cb_ );
    state_machine::test_fsm_ machine;
   
    ros::Rate loopRate(10);
    // bool machine_on = false;

    // if(!machine_on){
        machine.start();
        // machine_on=true;
        machine.process_event(state_machine::CmdTakeOff(nh)); state_machine::curr_state(machine);
        // machine.process_event(state_machine::CmdTrajectory()); state_machine::curr_state(machine);
        // machine.process_event(state_machine::CmdHover()); state_machine::curr_state(machine);
        machine.process_event(state_machine::CmdGetPkg(nh)); state_machine::curr_state(machine);
        // while(!machine.ReachedLZ(state_machine::CmdHover(nh)))
        // {
            machine.process_event(state_machine::CmdHover(nh)); state_machine::curr_state(machine);
        // }
    // }
    // if(machine_on && state_machine::AtLZ){
        // // machine.process_event(state_machine::CmdHover()); state_machine::curr_state(machine);
        machine.process_event(state_machine::CmdDescent(nh)); state_machine::curr_state(machine);
        // machine.process_event(state_machine::CmdLand()); state_machine::curr_state(machine);
        // machine.process_event(state_machine::CmdTakeOff(nh)); state_machine::curr_state(machine);
        // machine.process_event(state_machine::CmdGotoDrop(nh)); state_machine::curr_state(machine);
        // machine.process_event(state_machine::CmdHover()); state_machine::curr_state(machine);
        // machine.process_event(state_machine::CmdDescent()); state_machine::curr_state(machine);
        // machine.process_event(state_machine::CmdDrop()); state_machine::curr_state(machine);
        // machine.process_event(state_machine::CmdDropOver()); state_machine::curr_state(machine);
        // machine.process_event(state_machine::CmdAscent()); state_machine::curr_state(machine);
        // machine.process_event(state_machine::CmdGetPkg(nh)); state_machine::curr_state(machine);
        // machine.process_event(state_machine::CmdHover());state_machine::curr_state(machine);
        // machine.process_event(state_machine::CmdDescent()); state_machine::curr_state(machine);
        // machine.process_event(state_machine::CmdLand()); state_machine::curr_state(machine);
        std::cout<<"End Mission"<<std::endl;
        machine.stop();
        // machine_on = false;
    // }
    return 0;
}