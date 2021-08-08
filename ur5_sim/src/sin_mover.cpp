#include "ros/ros.h"
#include "std_msgs/Float64.h"

// Total number of Joints
const int Joints = 6;

// Loop Rate
const int loop_rate_val = 100;

int main(int argc, char **argv)
{
	ROS_INFO("Initialising sin_mover Node.....");
	// Initialise Sine-Mover Node
	ros::init(argc, argv, "sin_mover");
	
	ROS_INFO("Initialised the node.");
	// Create a handle to sine_mover node
	ros::NodeHandle n;

	// Set loop frequency
	ros::Rate loop_rate(loop_rate_val);

	//Create publishers to send position commands to all joints
	ros::Publisher joint_com_pub[Joints]; 
	joint_com_pub[0] = n.advertise<std_msgs::Float64>("/shoulder_pan_joint_position_controller/command", 1000);
	joint_com_pub[1] = n.advertise<std_msgs::Float64>("/shoulder_lift_joint_position_controller/command", 1000);
	joint_com_pub[2] = n.advertise<std_msgs::Float64>("/elbow_joint_position_controller/command", 1000);
	joint_com_pub[3] = n.advertise<std_msgs::Float64>("/wrist_1_joint_position_controller/command", 1000);
	joint_com_pub[4] = n.advertise<std_msgs::Float64>("/wrist_2_joint_position_controller/command", 1000);
	joint_com_pub[5] = n.advertise<std_msgs::Float64>("/wrist_3_joint_position_controller/command", 1000);

	int start_time, elapsed;

	// Get ROS start time
	while (not start_time) {
		start_time = ros::Time::now().toSec();
	}

	while (ros::ok()) {
		// Get ROS Elapsed Time
		elapsed = ros::Time::now().toSec() - start_time;

		// Set the arm joint angles
		std_msgs::Float64 joint1_angle, joint2_angle, joint3_angle, joint4_angle, joint5_angle, joint6_angle;
		joint1_angle.data = sin(2 * M_PI * 0.08 * elapsed) * (M_PI);
		joint2_angle.data = sin(2 * M_PI * 0.08 * elapsed) * (M_PI);
		joint3_angle.data = sin(2 * M_PI * 0.08 * elapsed) * (M_PI);
		joint4_angle.data = sin(2 * M_PI * 0.08 * elapsed) * (M_PI);
		joint5_angle.data = sin(2 * M_PI * 0.08 * elapsed) * (M_PI);
		joint6_angle.data = sin(2 * M_PI * 0.08 * elapsed) * (M_PI);

		// Publish the arm joint angles
		ROS_INFO("Publishing new angles..");
		joint_com_pub[0].publish(joint1_angle);
		joint_com_pub[1].publish(joint2_angle);
		joint_com_pub[2].publish(joint3_angle);
		joint_com_pub[3].publish(joint4_angle);
		joint_com_pub[4].publish(joint5_angle);
		joint_com_pub[5].publish(joint6_angle);
		
		ROS_INFO("Published new angles..");

		ros::spinOnce();

		// Sleep for the remaining time until 10 Hz is reached
		loop_rate.sleep();
	}	
	return 0;
}
