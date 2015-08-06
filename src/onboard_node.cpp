#include <ros/ros.h>
#include <rqt_quadcoptergui/onboardnodehandler.h>

//#define MULTITHREAD_ENABLE

#ifndef NUMBER_THREADS
#define NUMBER_THREADS 2
#endif

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "onboard_node");
  ros::NodeHandle nh;
  //Initialize Onboard NodeHandler:
  OnboardNodeHandler nodehandler_instance_(nh);

  //Start spinning:
#ifdef MULTITHREAD_ENABLE
  ros::MultiThreadedSpinner spinner(NUMBER_THREADS); // Use 4 threads
  spinner.spin();//Spin using multithreaded spinner
#else
  ros::spin();
#endif
}

