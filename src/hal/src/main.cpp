#include "turtle_pi_hal.h"

main()
{
  TurtlePiHal turtle_pi_robot;
  controller_manager::ControllerManager cm(&turtle_pi_robot);

  while (true)
  {
     turtle_pi_robot.read();
     cm.update(turtle_pi_robot.get_time(), turtle_pi_robot.get_period());
     turtle_pi_robot.write();
     sleep();
  }
}

