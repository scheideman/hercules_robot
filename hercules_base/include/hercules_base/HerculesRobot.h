#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <boost/assign.hpp>
#include <iostream>

class HerculesRobot : public hardware_interface::RobotHW
{
private:
	ros::NodeHandle nh_;

	sensor_msgs::JointState::ConstPtr feedback_msg;

	hardware_interface::JointStateInterface joint_state_interface_;
	hardware_interface::VelocityJointInterface velocity_joint_interface_;
	struct Joint
	{
		double position;
		double velocity;
		double effort;
		double velocity_command;

		Joint() : position(0), velocity(0), effort(0), velocity_command(0)
		{
		}
	}
	joints_[4];

	void feedbackCallback(const sensor_msgs::JointState::ConstPtr& msg);


public:
	HerculesRobot();
	ros::Subscriber feedback_sub_;
	ros::Publisher cmd_drive_pub;
	void read();
	void write();
};
