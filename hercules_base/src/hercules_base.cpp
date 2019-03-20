#include "controller_manager/controller_manager.h"
#include "hercules_base/HerculesRobot.h"
#include "ros/ros.h"

#include <boost/asio/io_service.hpp>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>

typedef boost::chrono::steady_clock time_source;

typedef boost::chrono::steady_clock time_source;

void controlThread(ros::Rate rate,HerculesRobot* hercules, controller_manager::ControllerManager* cm)
{
  time_source::time_point last_time = time_source::now();

  while (1)
  {
    // Calculate monotonic time elapsed
    // time_source::time_point this_time = time_source::now();
    // boost::chrono::duration<double> elapsed_duration = this_time - last_time;
    // ros::Duration elapsed(elapsed_duration.count());
    // last_time = this_time;

    // robot->copyJointsFromHardware();
    // cm->update(ros::Time::now(), elapsed);
    // robot->publishDriveFromController();
    // rate.sleep();

       // Calculate monotonic time elapsed
      time_source::time_point this_time = time_source::now();
      boost::chrono::duration<double> elapsed_duration = this_time - last_time;
      ros::Duration elapsed(elapsed_duration.count());
      last_time = this_time;

      hercules->read();
      cm->update(ros::Time::now(), elapsed);
      hercules->write();
      ros::spinOnce();
      rate.sleep();
  }
}



int main(int argc, char* argv[])
{
  // Initialize ROS node.
  ros::init(argc, argv, "hercules_node");
  ros::NodeHandle controller_nh("");
  HerculesRobot hercules;
  controller_manager::ControllerManager cm(&hercules, controller_nh);
  ros::Rate rate(50);
  
  boost::thread(boost::bind(controlThread, ros::Rate(50),&hercules, &cm));
  ros::spin();

  return 0;
}
