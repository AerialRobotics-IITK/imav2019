#include "planner_test.h"

int main(int argc, char **argv)
{
    ros::init(argc,argv,"planner_test");

    ros::NodeHandle nh;

    ros::Subscriber mav_pose_sub = nh.subscribe("odometry", 1, mav_pose_cb_);

    ros::Subscriber gripper_status_sub = nh.subscribe("gripper_status", 1,gripper_status_cb_ );
    state_machine::test_fsm_ machine;
   
    ros::Rate loopRate(0.2);

    machine.start();
    machine.process_event(state_machine::CmdTakeOff(nh)); state_machine::curr_state(machine);
    // machine.process_event(state_machine::CmdHover(nh)); state_machine::curr_state(machine);
    // machine.process_event(state_machine::CmdAscent(nh)); state_machine::curr_state(machine);
    // machine.process_event(state_machine::CmdTrajectory()); state_machine::curr_state(machine);
    // machine.process_event(state_machine::CmdHover(nh)); state_machine::curr_state(machine);
    // machine.process_event(state_machine::CmdGetPkg(nh)); state_machine::curr_state(machine);
    // machine.process_event(state_machine::CmdHover(nh)); state_machine::curr_state(machine);
    // machine.process_event(state_machine::CmdDescent(nh)); state_machine::curr_state(machine);
    // machine.process_event(state_machine::CmdLand(nh)); state_machine::curr_state(machine);
    // machine.process_event(state_machine::CmdTakeOff(nh)); state_machine::curr_state(machine);
    // machine.process_event(state_machine::CmdHover(nh)); state_machine::curr_state(machine);
    // machine.process_event(state_machine::CmdAscent(nh)); state_machine::curr_state(machine);
    loopRate.sleep();
    machine.process_event(state_machine::CmdGotoDrop(nh)); state_machine::curr_state(machine);
    loopRate.sleep();
    machine.process_event(state_machine::CmdHover(nh)); state_machine::curr_state(machine);
    loopRate.sleep();
    machine.process_event(state_machine::CmdDescent(nh)); state_machine::curr_state(machine);
    loopRate.sleep();
    machine.process_event(state_machine::CmdDrop(nh)); state_machine::curr_state(machine);
    loopRate.sleep();
    machine.process_event(state_machine::CmdDropOver(nh)); state_machine::curr_state(machine);
    loopRate.sleep();
    machine.process_event(state_machine::CmdAscent(nh)); state_machine::curr_state(machine);
    loopRate.sleep();
    machine.process_event(state_machine::CmdGetPkg(nh)); state_machine::curr_state(machine);
    loopRate.sleep();
    machine.process_event(state_machine::CmdHover(nh));state_machine::curr_state(machine);
    loopRate.sleep();
    machine.process_event(state_machine::CmdDescent(nh)); state_machine::curr_state(machine);
    loopRate.sleep();
    machine.process_event(state_machine::CmdLand(nh)); state_machine::curr_state(machine);
    std::cout<<"End Mission"<<std::endl;
    machine.stop();
    // machine_on = false;
// }
return 0;
}