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
#include <std_msgs/Empty.h>
#include <string>
#include <swiftpro/SwiftproState.h>

serial::Serial _serial;				// serial object
float cart[3] = {0.0};				// 3 cartesian coordinates: x, y, z(mm)

/* 
 * Description: callback when receive data from cart_rece_topic
 * Inputs: 		msg					3 cartesian coordinates: x, y, z(mm)
 * Outputs:		Gcode				send gcode to control swift pro
 */
void cart_callback(const swiftpro::SwiftproState& msg)
{
	std::string Gcode = "";
	std_msgs::String result;
	char x[10];
	char y[10];
	char z[10];

	// ROS_INFO("Cart: %f, %f, %f", msg.cart_x, msg.cart_y, msg.cart_z);
	sprintf(x, "%.2f", msg.cart_x);
	sprintf(y, "%.2f", msg.cart_y);
	sprintf(z, "%.2f", msg.cart_z);
	Gcode = (std::string)"G0 X" + x + " Y" + y + " Z" + z + " F10000" + "\r\n";
	ROS_INFO("%s", Gcode.c_str());
	_serial.write(Gcode.c_str());
	result.data = _serial.read(_serial.available());
}

/* 
 * Node name:
 *	 serial_send_node
 *
 * Topic subscribe: (queue size = 1)
 *	 cart_send_topic
 */
int main(int argc, char** argv)
{	
	ros::init(argc, argv, "serial_send_node");
	ros::NodeHandle nh;
	swiftpro::SwiftproState swiftpro_state;
	
	ros::Subscriber sub = nh.subscribe("cart_send_topic", 1, cart_callback);

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
		ros::Duration(3.5).sleep();				// wait 3s
		_serial.write("M2120 V0\r\n");			// stop report position
		ros::Duration(0.1).sleep();				// wait 0.1s
		_serial.write("M17\r\n");				// attach
		ros::Duration(0.1).sleep();				// wait 0.1s
		ROS_INFO_STREAM("Attach and wait for commands");
	}

	ros::spin();
}


