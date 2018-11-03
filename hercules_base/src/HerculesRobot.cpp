#include <boost/assign.hpp>
#include "hercules_base/HerculesRobot.h"

HerculesRobot::HerculesRobot() : nh_()
{
  ros::V_string joint_names = boost::assign::list_of("front_left_wheel_joint")
	  ("front_right_wheel_joint")("rear_left_wheel_joint")("rear_right_wheel_joint");

  for (unsigned int i = 0; i < joint_names.size(); i++) {
	  hardware_interface::JointStateHandle joint_state_handle(joint_names[i],
			   &joints_[i].position, &joints_[i].velocity, &joints_[i].effort);
	  	  	   joint_state_interface_.registerHandle(joint_state_handle);

	  hardware_interface::JointHandle joint_handle(
		      joint_state_handle, &joints_[i].velocity_command);
	  velocity_joint_interface_.registerHandle(joint_handle);
  }
  registerInterface(&joint_state_interface_);
  registerInterface(&velocity_joint_interface_);

  feedback_sub_ = nh_.subscribe("/hw_feedback", 1, &HerculesRobot::feedbackCallback,this);
  cmd_drive_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_drive", 1);
}

void HerculesRobot::read()
{
  if (feedback_msg)
  {
    for (int i = 0; i < 4; i++)
    {
      joints_[i].position = feedback_msg->position[i];
      joints_[i].velocity = feedback_msg->velocity[i];
      joints_[i].effort = 0;
    }
  }
}


void HerculesRobot::write()
{

	double left = joints_[0].velocity_command;
	double right = joints_[1].velocity_command;

	std::cout <<  left << std::endl;


}

void HerculesRobot::feedbackCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  feedback_msg = msg;
}
