#include "controller_manager/controller_manager.h"
#include "hercules_base/HerculesRobot.h"
#include "ros/ros.h"

typedef boost::chrono::steady_clock time_source;


int main(int argc, char* argv[])
{
  // Initialize ROS node.
  ros::init(argc, argv, "hercules_node");
  HerculesRobot hercules;
  controller_manager::ControllerManager cm(&hercules);
  ros::Rate rate(50);
  time_source::time_point last_time = time_source::now();
   while (ros::ok())
   {

      // Calculate monotonic time elapsed
      time_source::time_point this_time = time_source::now();
      boost::chrono::duration<double> elapsed_duration = this_time - last_time;
      ros::Duration elapsed(elapsed_duration.count());
      last_time = this_time;

      hercules.read();
      cm.update(ros::Time::now(), elapsed);
      hercules.write();
      ros::spinOnce();
      rate.sleep();
   }

  return 0;
}
