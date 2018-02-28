#include "turtle_pi_hal.h"

TurtlePiHal::TurtlePiHal()
{ 
    // connect and register the joint state interface
    hardware_interface::JointStateHandle state_handle_left("left_wheel_joint", &pos[0], &vel[0], &eff[0]);
    jnt_state_interface.registerHandle(state_handle_left);

    hardware_interface::JointStateHandle state_handle_right("right_wheel_joint", &pos[1], &vel[1], &eff[1]);
    jnt_state_interface.registerHandle(state_handle_right);

    registerInterface(&jnt_state_interface);

    // connect and register the joint position interface
    hardware_interface::JointHandle pos_handle_left(jnt_state_interface.getHandle("left_wheel_joint"), &cmd[0]);
    jnt_pos_interface.registerHandle(pos_handle_left);

    hardware_interface::JointHandle pos_handle_right(jnt_state_interface.getHandle("right_wheel_joint"), &cmd[1]);
    jnt_pos_interface.registerHandle(pos_handle_right);

    registerInterface(&jnt_pos_interface);
}


void TurtlePiHal::read() {
    printf("HAL: Reading - leftcmd: %0.2f; rightcmd: %0.2f\n", cmd[0], cmd[1]);
}

void TurtlePiHal::write() {
    printf("HAL: write\n");
    printf("Right pve = %0.2f, %0.2f, %0.2f\n", pos[1], vel[1], eff[1]);
}