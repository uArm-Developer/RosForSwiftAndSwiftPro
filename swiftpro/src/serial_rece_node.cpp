/* 
 * Software License Agreement (BSD License)
 * Copyright (c) 2017, UFactory, Inc.
 * All rights reserved.
 * Author: Roger Cui  <roger@ufactory.cc>
 *		   David Long <xiaokun.long@ufactory.cc>	   
 */
 
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <string>
#include <swiftpro/SwiftproState.h>

serial::Serial _serial;				// serial object
float cart[3] = {0.0};				// 3 cartesian coordinates: x, y, z(mm)
char  strdata[2048];				// global variables for handling string


void handlestr()
{
	char  *pch = strtok(strdata, " ");
	float value[8];
	int   index = 0;

	while (pch != NULL && index < 5)
	{
		value[index] = atof(pch+1);
		pch = strtok(NULL, " ");
		index++;
	}
	cart[0] = value[1];
	cart[1] = value[2];
	cart[2] = value[3];
}


void handlechar(char c)
{
	static int index = 0;

	switch(c)
	{
		case '\r':
			break;	

		case '\n':
			strdata[index] = '\0';
			handlestr();
			index = 0;
			break;

		default:
			strdata[index++] = c;
			break;
	}
}

/* 
 * Node name:
 *	 serial_rece_node
 *
 * Topic publish: (rate = 20Hz, queue size = 1)
 *   cart_rece_topic
 */
int main(int argc, char** argv)
{	
	ros::init(argc, argv, "serial_rece_node");
	ros::NodeHandle nh;
	swiftpro::SwiftproState swiftpro_state;
	std_msgs::String result;

	ros::Publisher pub = nh.advertise<swiftpro::SwiftproState>("cart_rece_topic", 1);
	ros::Rate loop_rate(20);

	try
	{
		_serial.setPort("/dev/ttyACM0");
		_serial.setBaudrate(115200);
		serial::Timeout to = serial::Timeout::simpleTimeout(1000);
		_serial.setTimeout(to);
		_serial.open();
		ROS_INFO_STREAM("Port has been open successfully");
	}
	catch (serial::IOException& e)
	{
		ROS_ERROR_STREAM("Unable to open port");
		return -1;
	}
	
	if (_serial.isOpen())
	{
		ros::Duration(3.0).sleep();				// wait 3s
		_serial.write("M2019\r\n");				// detach
		ros::Duration(0.5).sleep();				// wait 0.5s
		_serial.write("M2120 V0.05\r\n");		// report position per 0.05s
		ROS_INFO_STREAM("Start to report data");
	}
	
	while (ros::ok())							// publish cartesian coordinates
	{
		if (_serial.available())
		{
			result.data = _serial.read(_serial.available());
			// ROS_INFO_STREAM("Read:" << result.data);
			for (int i = 0; i < result.data.length(); i++)
				handlechar(result.data.c_str()[i]);

			swiftpro_state.motor_angle1 = 0.0;
			swiftpro_state.motor_angle2 = 0.0;
			swiftpro_state.motor_angle3 = 0.0;
			swiftpro_state.motor_angle4 = 0.0;
			swiftpro_state.cart_x = cart[0];
			swiftpro_state.cart_y = cart[1];
			swiftpro_state.cart_z = cart[2];
			pub.publish(swiftpro_state);
			ROS_INFO("Cart: %f %f %f", cart[0], cart[1], cart[2]);
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
}


