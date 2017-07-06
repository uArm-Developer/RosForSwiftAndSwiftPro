/* 
 * Software License Agreement (BSD License)
 * Copyright (c) 2017, UFactory, Inc.
 * All rights reserved.
 * Author: Roger Cui  <roger@ufactory.cc>
 *		   David Long <xiaokun.long@ufactory.cc>	   
 */

#include <string>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <swiftpro/SwiftproState.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

#define MATH_PI 				3.141592653589793238463
#define MATH_TRANS  			57.2958    
#define MATH_L1 				106.6
#define MATH_L2 				13.2
#define MATH_LOWER_ARM 			142.07
#define MATH_UPPER_ARM 			158.81	
#define MATH_UPPER_LOWER 		(MATH_UPPER_ARM / MATH_LOWER_ARM)

#define LOWER_ARM_MAX_ANGLE     135.6
#define LOWER_ARM_MIN_ANGLE     0
#define UPPER_ARM_MAX_ANGLE     100.7
#define UPPER_ARM_MIN_ANGLE     0
#define LOWER_UPPER_MAX_ANGLE   151
#define LOWER_UPPER_MIN_ANGLE   10

float joint_angle[9] = {0.0};		// 9 joint angles of swiftpro(degree)

/* 
 * Description: Get 9 joint angles from 3 motor angles
 * Inputs: 		angle[3]			3 motor angles(degree)
 * Outputs:		joint_angle[9]		9 joint angles(degree)
 */
void all_joints_state(float angle[3])
{
	double alpha2;
	double alpha3;
	
	alpha2 = angle[1];
	alpha3 = angle[2] - 3.8;
	
	// 3 necessary joints for kinematic chain
	joint_angle[0] = angle[0] - 90;
	joint_angle[1] = 90 - alpha2;
	joint_angle[5] = alpha3;

	// 6 passive joints for display
	joint_angle[2] = (alpha2 + alpha3) - 176.11 + 90;
	joint_angle[3] = -90 + alpha2;
	joint_angle[4] = joint_angle[1];
	joint_angle[6] = 90 - (alpha2 + alpha3);
	joint_angle[7] = 176.11 - 180 - alpha3;
	joint_angle[8] = 48.39 + alpha3 - 44.55;
}

/* 
 * Description: forward kinematics of swift pro
 * Inputs: 		angle[3]			3 motor angles(degree)
 * Outputs:		cart[3]				3 cartesian coordinates: x, y, z(mm)
 */
void swift_fk(float angle[3], float cart[3])
{
	double stretch = MATH_LOWER_ARM * cos(angle[1] / MATH_TRANS) 
				   + MATH_UPPER_ARM * cos(angle[2] / MATH_TRANS) + MATH_L2 + 56.55;

	double height = MATH_LOWER_ARM * sin(angle[1] / MATH_TRANS) 
				  - MATH_UPPER_ARM * sin(angle[2] / MATH_TRANS) + MATH_L1;
	
	cart[0] = stretch * sin(angle[0] / MATH_TRANS);
	cart[1] = -stretch * cos(angle[0] / MATH_TRANS);
	cart[2] = height - 74.55;
}

/* 
 * Description: inverse kinematics of swift pro
 * Inputs: 		cart[3]				3 cartesian coordinates: x, y, z(mm)
 * Outputs:		angle[3]			3 motor angles(degree)
 */
bool swiftpro_ik(float cart[3], float angle[3])
{
	float x = cart[0];
	float y = cart[1];
	float z = cart[2];
	float xIn, zIn, phi, rightAll, sqrtZX = 0.0;
	float angleRot, angleLeft, angleRight = 0.0;
	
	z += 74.55;
	zIn = (z - MATH_L1) / MATH_LOWER_ARM;
	
	if (x < 0.1)
		x = 0.1;

	// calculate value of theta1: the rotation angle
	if (y == 0)
		angleRot = 90;
	else if (y < 0)
		angleRot = -atan(x / y) * MATH_TRANS;
	else if (y > 0)
		angleRot = 180 - atan(x / y) * MATH_TRANS;

	xIn 	= (x / sin(angleRot / MATH_TRANS) - MATH_L2 - 56.55) / MATH_LOWER_ARM;
	phi 	= atan(zIn / xIn) * MATH_TRANS;
	sqrtZX 	= sqrt(zIn * zIn + xIn * xIn);
	rightAll   = (sqrtZX * sqrtZX + MATH_UPPER_LOWER * MATH_UPPER_LOWER  - 1) 
			   / (2 * MATH_UPPER_LOWER  * sqrtZX);
	angleRight = acos(rightAll) * MATH_TRANS;

	// calculate value of theta2 and theta3
	rightAll   = (sqrtZX * sqrtZX + 1 - MATH_UPPER_LOWER * MATH_UPPER_LOWER ) / (2 * sqrtZX);
	angleLeft  = acos(rightAll) * MATH_TRANS;
	angleLeft  = angleLeft + phi;
	angleRight = angleRight - phi;

	if (isnan(angleRot) || isnan(angleLeft) || isnan(angleRight))
		return false;

	angle[0] = angleRot;
	angle[1] = angleLeft;
	angle[2] = angleRight;
	return true;
}

/* 
 * Description: callback when receive data from cart_send_topic
 * Inputs: 		msg					3 cartesian coordinates: x, y, z(mm)
 * Outputs:		joint_angle[9]		9 joint angles(degree)
 */
void cart_Callback(const swiftpro::SwiftproState& msg)
{
	float cart[3];
	float angle[3];
	
	cart[0] = msg.cart_x;
	cart[1] = msg.cart_y;
	cart[2] = msg.cart_z;
	
	if ( swiftpro_ik(cart, angle) )
		all_joints_state(angle);
	else
		ROS_ERROR("Inverse kinematic is wrong");
}

/* 
 * Description: callback when receive data from move_group/fake_controller_joint_states
 * Inputs: 		msg					3 necessary joints for kinematic chain(degree)
 * Outputs:		joint_angle[9]		9 joint angles(degree)
 */
void joint_Callback(const sensor_msgs::JointState& msg)
{
	double alpha2;
	double alpha3;
	
	joint_angle[0] = msg.position[0] * 57.2958;
	joint_angle[1] = msg.position[1] * 57.2958;
	joint_angle[2] = msg.position[2] * 57.2958;
	
	alpha2 = 90 - joint_angle[1];
	alpha3 = joint_angle[2] - alpha2  + 176.11 - 90;
	
	joint_angle[3] = -90 + alpha2;
	joint_angle[4] = joint_angle[1];
	joint_angle[5] = alpha3;
	joint_angle[6] = 90 - (alpha2 + alpha3);
	joint_angle[7] = 176.11 - 180 - alpha3;
	joint_angle[8] = 48.39 + alpha3 - 44.55;
}


/* 
 * Node name:
 *	 state_node
 *
 * Topic publish: (rate = 20Hz, queue size = 1)
 *   cart_send_topic
 *	 joint_states
 *
 * Topic subscribe: (queue size = 1)
 *	 cart_rece_topic
 *   move_group/fake_controller_joint_states
 *
 * Parameters:
 *   None
 */
int main(int argc, char **argv)
{
	float angle[3];
	float cart[3];
	
	ros::init(argc, argv, "state_node");
	ros::NodeHandle n;
	
	ros::Subscriber cart_sub  = n.subscribe("cart_rece_topic", 1, cart_Callback);
	ros::Subscriber joint_sub = n.subscribe("move_group/fake_controller_joint_states", 1, joint_Callback);
	ros::Publisher 	cart_pub  = n.advertise<swiftpro::SwiftproState>("cart_send_topic", 1);
	ros::Publisher 	joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
	ros::Rate loop_rate(20);

	tf::TransformBroadcaster 		broadcaster;
	sensor_msgs::JointState 		joint_state;
	geometry_msgs::TransformStamped odom_trans;
	swiftpro::SwiftproState			swiftpro_state;
	
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id  = "Base";
	
	while (ros::ok())
	{
		joint_state.header.stamp = ros::Time::now();
		joint_state.name.resize(9);
		joint_state.position.resize(9);
		joint_state.name[0] = "Joint1";
		joint_state.position[0] = joint_angle[0] / 57.2958;
	    joint_state.name[1] = "Joint2";
		joint_state.position[1] = joint_angle[1] / 57.2958;
		joint_state.name[2] = "Joint3";
		joint_state.position[2] = joint_angle[2] / 57.2958;
		joint_state.name[3] = "Joint4";
		joint_state.position[3] = joint_angle[3] / 57.2958;
		joint_state.name[4] = "Joint5";
		joint_state.position[4] = joint_angle[4] / 57.2958;
		joint_state.name[5] = "Joint6";
		joint_state.position[5] = joint_angle[5] / 57.2958;
		joint_state.name[6] = "Joint7";
		joint_state.position[6] = joint_angle[6] / 57.2958;
		joint_state.name[7] = "Joint8";
		joint_state.position[7] = joint_angle[7] / 57.2958;
		joint_state.name[8] = "Joint9";
		joint_state.position[8] = joint_angle[8] / 57.2958;
		
		angle[0] = joint_angle[0] + 90;
		angle[1] = 90 - joint_angle[1];
		angle[2] = joint_angle[5] + 3.8;
		swift_fk(angle, cart);
		swiftpro_state.motor_angle1 = angle[0];
		swiftpro_state.motor_angle2 = angle[1];
		swiftpro_state.motor_angle3 = angle[2];
		swiftpro_state.motor_angle4 = 0.0;
		swiftpro_state.cart_x = cart[0];
		swiftpro_state.cart_y = cart[1];
		swiftpro_state.cart_z = cart[2];
		ROS_INFO("x = %f, y = %f, z = %f", cart[0], cart[1], cart[2]);

		odom_trans.header.stamp = ros::Time::now();
		odom_trans.transform.translation.x = 0;
		odom_trans.transform.translation.y = 0;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(10);
		
		joint_pub.publish(joint_state);
		cart_pub.publish(swiftpro_state);
		broadcaster.sendTransform(odom_trans);
		
		ros::spinOnce();
		loop_rate.sleep();
	}		
	return 0;
}

