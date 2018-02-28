#include "turtle_pi_hal.h"

TurtlePiHal::TurtlePiHal()
{ 
    // connect and register the joint state interface
    hardware_interface::JointStateHandle state_handle_left("left", &pos[0], &vel[0], &eff[0]);
    jnt_state_interface.registerHandle(state_handle_left);

    hardware_interface::JointStateHandle state_handle_right("right", &pos[1], &vel[1], &eff[1]);
    jnt_state_interface.registerHandle(state_handle_right);

    registerInterface(&jnt_state_interface);

    // connect and register the joint position interface
    hardware_interface::JointHandle pos_handle_left(jnt_state_interface.getHandle("left"), &cmd[0]);
    jnt_pos_interface.registerHandle(pos_handle_left);

    hardware_interface::JointHandle pos_handle_right(jnt_state_interface.getHandle("right"), &cmd[1]);
    jnt_pos_interface.registerHandle(pos_handle_right);

    registerInterface(&jnt_pos_interface);
}
