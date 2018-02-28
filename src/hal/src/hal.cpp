#include "turtle_pi_hal.h"
#include <controller_manager/controller_manager.h>
#include <unistd.h>
#include <signal.h>
#include <sensor_msgs/JointState.h>
#include <ros/callback_queue.h>

#define MILLISECONDS 1000

bool g_quit = false;

void quitRequested(int sig) {
  g_quit = true;
}

void SetJointStates(const sensor_msgs::JointState::ConstPtr &_js)
{
  static ros::Time startTime = ros::Time::now();

  printf("In callback = js: %s\n", _js->name[0].c_str());
}

void subscribeToJointStates(ros::NodeHandle *nh) {
 // ros topic subscriptions
  ros::SubscribeOptions jointStatesSo = ros::SubscribeOptions::create<sensor_msgs::JointState>(
    "/joint_states", 1, SetJointStates,
    ros::VoidPtr(), nh->getCallbackQueue());

  // Because TCP causes bursty communication with high jitter,
  // declare a preference on UDP connections for receiving
  // joint states, which we want to get at a high rate.
  // Note that we'll still accept TCP connections for this topic
  // (e.g., from rospy nodes, which don't support UDP);
  // we just prefer UDP.
  jointStatesSo.transport_hints = ros::TransportHints().unreliable();

  ros::Subscriber subJointStates = nh->subscribe(jointStatesSo);
  printf("Subscribed to JointStates\n");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "turtle_pi_hal");
  ros::NodeHandle nh;

  ros::CallbackQueue queue;
  nh.setCallbackQueue(&queue);

  ros::AsyncSpinner spinner(4, &queue);
  spinner.start();

  // Add custom signal handlers
  signal(SIGTERM, quitRequested);
  signal(SIGINT, quitRequested);
  signal(SIGHUP, quitRequested);

  TurtlePiHal turtle_pi_robot;
  controller_manager::ControllerManager cm(&turtle_pi_robot, nh);

  subscribeToJointStates(&nh);

  ros::Time ts = ros::Time::now();
  ros::Rate rate(3);
  //while (ros::ok())
  while (!g_quit)
  {
    ros::Duration d = ros::Time::now() - ts;
    ts = ros::Time::now();

    turtle_pi_robot.read();
    cm.update(ts, d);
    turtle_pi_robot.write();

    rate.sleep();
  }

  spinner.stop();

	exit(0);
}
