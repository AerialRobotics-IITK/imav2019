#include "planner.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "planner");
    ros::NodeHandle nh;
    ros::Rate loopRate(0.5);    
    state_machine::fsm_ machine;

    machine.start();        machine.process_event(state_machine::CmdTakeOff(nh));       state_machine::curr_state(machine);

    while(state_machine::ContMission)
    {

        loopRate.sleep();       machine.process_event(state_machine::CmdExplored(nh));     state_machine::curr_state(machine);
        loopRate.sleep();       machine.process_event(state_machine::CmdHover(nh));         state_machine::curr_state(machine);

        if(state_machine::PkgAttached)
        {

            loopRate.sleep();       machine.process_event(state_machine::CmdGotoDrop(nh));      state_machine::curr_state(machine); 
            loopRate.sleep();       machine.process_event(state_machine::CmdHover(nh));         state_machine::curr_state(machine);
            loopRate.sleep();       machine.process_event(state_machine::CmdDescent(nh));       state_machine::curr_state(machine);
            loopRate.sleep();       machine.process_event(state_machine::CmdDrop(nh));          state_machine::curr_state(machine);
            loopRate.sleep();       machine.process_event(state_machine::CmdDropOver(nh));      state_machine::curr_state(machine);
            loopRate.sleep();       machine.process_event(state_machine::CmdAscent(nh));        state_machine::curr_state(machine);

        }
    }
        
    loopRate.sleep();       machine.process_event(state_machine::CmdGotoLZ(nh));        state_machine::curr_state(machine);
    loopRate.sleep();       machine.process_event(state_machine::CmdHover(nh));         state_machine::curr_state(machine);
    loopRate.sleep();       machine.process_event(state_machine::CmdDescent(nh));       state_machine::curr_state(machine);
    loopRate.sleep();       machine.process_event(state_machine::CmdLand(nh));          state_machine::curr_state(machine);
        
    machine.stop();
    return 0;
}