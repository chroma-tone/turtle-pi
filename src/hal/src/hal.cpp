#include "turtle_pi_hal.h"
#include <controller_manager/controller_manager.h>
#include <unistd.h>
#include <signal.h>

#define MILLISECONDS 1000

bool g_quit = false;

void quitRequested(int sig) {
  g_quit = true;
}

int main(int argc, char **argv)
{
  ros::Duration elapsed_time;
  struct timespec last_time, current_time;
  static const double BILLION = 1000000000.0;

  ros::init(argc, argv, "turtle_pi_hal");
  ros::NodeHandle nh;

  // Add custom signal handlers
  signal(SIGTERM, quitRequested);
  signal(SIGINT, quitRequested);
  signal(SIGHUP, quitRequested);

  TurtlePiHal turtle_pi_robot;
  controller_manager::ControllerManager cm(&turtle_pi_robot, nh);

  clock_gettime(CLOCK_MONOTONIC, &last_time);
  ros::Rate r(3);

  while (!g_quit)
  {
    turtle_pi_robot.read();

    // Control
    clock_gettime(CLOCK_MONOTONIC, &current_time);
    elapsed_time = ros::Duration(current_time.tv_sec - last_time.tv_sec + (current_time.tv_nsec - last_time.tv_nsec) / BILLION);
    ros::Time ros_time = ros::Time::now();

    cm.update(ros_time, elapsed_time);
    //cm.update(turtle_pi_robot.get_time(), turtle_pi_robot.get_period());
    last_time = current_time;

    turtle_pi_robot.write();

    ros::spinOnce();
    r.sleep();
  }

	exit(0);
}
