#include <imav_planner/planner.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "planner");
    ros::NodeHandle nh, ph("~");

    ph.getParam("verbose", verbose);
    
    ph.getParam("error/gps", gps_error);
    ph.getParam("error/local", loc_error);

    ph.getParam("height/hover", hover_height);
    ph.getParam("height/drop", drop_height);
    ph.getParam("height/land", land_height);
    ph.getParam("height/step", descent_step);
    
    ph.getParam("servo/open", open_angle);
    ph.getParam("servo/close", close_angle);
    ph.getParam("servo/static", eq_angle);

    ph.getParam("delay/hover", hover_time);
    ph.getParam("delay/transition", transition_time);
    ph.getParam("delay/pitStop", wait_time);

    ros::Rate transitRate(1.0/transition_time);    
    state_machine::fsm_ machine;

    machine.start();    

    auto state = std::async(std::launch::async, state_machine::statePublish, ph, &machine);

    machine.process_event(state_machine::CmdTakeOff(nh));       
    
    if(verbose) state_machine::echo_state(machine);

// /*
    // Execution loop

    while(state_machine::ContMission)
    {
        transitRate.sleep();       
        machine.process_event(state_machine::CmdExploring(nh));    
        if(verbose)   state_machine::echo_state(machine);
        
        transitRate.sleep();       
        machine.process_event(state_machine::CmdHover(nh));        
        if(verbose)   state_machine::echo_state(machine);

        if(state_machine::PkgAttached)
        {   
            transitRate.sleep();       
            machine.process_event(state_machine::CmdGotoDrop(nh));      
            if(verbose)   state_machine::echo_state(machine); 
            
            transitRate.sleep();       
            machine.process_event(state_machine::CmdHover(nh));         
            if(verbose)   state_machine::echo_state(machine);
            
            transitRate.sleep();       
            machine.process_event(state_machine::CmdDescent(nh));       
            if(verbose)   state_machine::echo_state(machine);
            
            transitRate.sleep();       
            machine.process_event(state_machine::CmdDrop(nh));          
            if(verbose)   state_machine::echo_state(machine);
            
            transitRate.sleep();       
            machine.process_event(state_machine::CmdDropOver(nh));      
            if(verbose)   state_machine::echo_state(machine);
            
            transitRate.sleep();       
            machine.process_event(state_machine::CmdAscent(nh));        
            if(verbose)   state_machine::echo_state(machine);

        }
    
    }
        
    // transitRate.sleep();       
    // machine.process_event(state_machine::CmdGotoLZ(nh));      
    // if(verbose)   state_machine::echo_state(machine);
    
    // transitRate.sleep();       
    // machine.process_event(state_machine::CmdHover(nh));       
    // if(verbose)   state_machine::echo_state(machine);
    
    transitRate.sleep();       
    machine.process_event(state_machine::CmdDescent(nh));     
    if(verbose)   state_machine::echo_state(machine);
    
    transitRate.sleep();       
    machine.process_event(state_machine::CmdLand(nh));        
    if(verbose)   state_machine::echo_state(machine);

                                                                                                                                        // */
/* 
    // Landing test
    
    transitRate.sleep();       
    machine.process_event(state_machine::CmdDescent(nh));     
    if(verbose)   state_machine::echo_state(machine);
    state_machine::ContMission = false;
    
    transitRate.sleep();       
    machine.process_event(state_machine::CmdLand(nh));        
    if(verbose)   state_machine::echo_state(machine);

                                                                                                                                        */
    machine.stop();
    return 0;
}
