#include<imav_planner/planner.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "planner2");
    ros::NodeHandle nh;

    nh.getParam("planner2/height/hover", hover_height);
    nh.getParam("planner2/height/drop", drop_height);
    nh.getParam("planner2/height/land", land_height);

    nh.getParam("planner2/servo/open", open_angle);
    nh.getParam("planner2/servo/close", close_angle);
    nh.getParam("planner2/servo/static", eq_angle);

    nh.getParam("planner2/delay/hover", hover_time);
    nh.getParam("planner2/delay/transition", transition_time);
    nh.getParam("planner2/delay/pitStop", wait_time);

    ros::Rate transitRate(1.0/transition_time);
    ros::Rate waitRate(1.0/wait_time);

    state_machine::fsm_ machine;

    machine.start();        machine.process_event(state_machine::CmdTakeOff(nh));       state_machine::curr_state(machine);

    //Execution Loop

    while (state_machine::ContMission)
    {
        transitRate.sleep();       machine.process_event(state_machine::CmdExploring(nh));     state_machine::curr_state(machine);
        transitRate.sleep();       machine.process_event(state_machine::CmdHover(nh));         state_machine::curr_state(machine);

        if(state_machine::PkgAttached)
        {

            transitRate.sleep();       machine.process_event(state_machine::CmdGotoDrop(nh));      state_machine::curr_state(machine); 
            transitRate.sleep();       machine.process_event(state_machine::CmdHover(nh));         state_machine::curr_state(machine);
            transitRate.sleep();       machine.process_event(state_machine::CmdDescent(nh));       state_machine::curr_state(machine);
            transitRate.sleep();       machine.process_event(state_machine::CmdDrop(nh));          state_machine::curr_state(machine);
            transitRate.sleep();       machine.process_event(state_machine::CmdDropOver(nh));      state_machine::curr_state(machine);
            transitRate.sleep();       machine.process_event(state_machine::CmdAscent(nh));        state_machine::curr_state(machine);
            
            if(!state_machine::PkgAttached)
            {
                state_machine::ContMission = false;
                
                transitRate.sleep();       machine.process_event(state_machine::CmdGotoLZ(nh));         state_machine::curr_state(machine);
                transitRate.sleep();       machine.process_event(state_machine::CmdHover(nh));         state_machine::curr_state(machine);
                transitRate.sleep();       machine.process_event(state_machine::CmdDescent(nh));       state_machine::curr_state(machine);
                transitRate.sleep();       machine.process_event(state_machine::CmdLand(nh));          state_machine::curr_state(machine);
                
                waitRate.sleep();
                
                transitRate.sleep();       machine.process_event(state_machine::CmdTakeOff(nh));       state_machine::curr_state(machine);
                
                if(state_machine::PkgAttached)
                {
                    transitRate.sleep();       machine.process_event(state_machine::CmdGotoDrop(nh));      state_machine::curr_state(machine); 
                    transitRate.sleep();       machine.process_event(state_machine::CmdHover(nh));         state_machine::curr_state(machine);
                    transitRate.sleep();       machine.process_event(state_machine::CmdDescent(nh));       state_machine::curr_state(machine);
                    transitRate.sleep();       machine.process_event(state_machine::CmdDrop(nh));          state_machine::curr_state(machine);
                    transitRate.sleep();       machine.process_event(state_machine::CmdDropOver(nh));      state_machine::curr_state(machine);
                    transitRate.sleep();       machine.process_event(state_machine::CmdAscent(nh));        state_machine::curr_state(machine);    
                }
                
                state_machine::ContMission = true;
            }
        }
    }

    transitRate.sleep();       machine.process_event(state_machine::CmdGotoLZ(nh));         state_machine::curr_state(machine);
    transitRate.sleep();       machine.process_event(state_machine::CmdHover(nh));         state_machine::curr_state(machine);
    transitRate.sleep();       machine.process_event(state_machine::CmdDescent(nh));       state_machine::curr_state(machine);
    transitRate.sleep();       machine.process_event(state_machine::CmdLand(nh));          state_machine::curr_state(machine);

    machine.stop();
    return 0;    
}