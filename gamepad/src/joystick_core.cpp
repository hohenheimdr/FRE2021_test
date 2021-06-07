#include "ros/ros.h"
#include <std_msgs/Int64.h>
#include <std_msgs/Bool.h>
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/TwistStamped.h"
#include "differential_ifk_lib/DifferentialIfk.hpp"
#include <iostream>

//dr 06.06.17 removed speed left and right for better performance



ros::Publisher pub_joystick_cmd_vel;
ros::Publisher pub_joystick_button_A,pub_joystick_button_B, pub_joystick_button_X, pub_joystick_button_Y;
ros::Publisher pub_joystick_button_LB,	pub_joystick_button_RB ,pub_joystick_button_LT,	pub_joystick_button_RT;
//publisher msgs for motorcontroller speed
geometry_msgs::TwistStamped cmd_vel;
std_msgs::Bool False, True;

DifferentialIfk differential;

void Joystick_callback(const sensor_msgs::Joy::ConstPtr& msg)
{
	False.data=false;
	True.data=true;
	cmd_vel.twist.linear.x=0;
	cmd_vel.twist.linear.y=0;
	cmd_vel.twist.angular.z=0;	
	
	cmd_vel.twist.linear.x=msg->axes[4];
	cmd_vel.twist.angular.z=msg->axes[3];
		
	cmd_vel.header.stamp=ros::Time::now();
	pub_joystick_cmd_vel.publish(cmd_vel);
	//adress the other joystick values
		if(msg->buttons[4]==1)
		{
			pub_joystick_button_LB.publish(True);
			pub_joystick_button_A.publish(False);
			pub_joystick_button_B.publish(False);
			pub_joystick_button_X.publish(False);
			pub_joystick_button_Y.publish(False);	
			pub_joystick_button_RB.publish(False);
			pub_joystick_button_LT.publish(False);
			pub_joystick_button_RT.publish(False);

		}else
		{
			pub_joystick_button_LB.publish(False);
		}
		//toggle button states when pressed
		if (msg->buttons[0]==1)
		{
			pub_joystick_button_A.publish(True);
		}
		if (msg->buttons[1]==1)
		{	
			pub_joystick_button_B.publish(True);
		}
		if (msg->buttons[2]==1)
		{	
			pub_joystick_button_X.publish(True);
		}
		if (msg->buttons[3]==1)
		{	
			pub_joystick_button_Y.publish(True);
		}
		
		if (msg->buttons[5]==1)
		{	
			pub_joystick_button_RB.publish(True);
		}else{
			pub_joystick_button_RB.publish(False);
		}
		
		if (msg->axes[2]<1)
		{
			pub_joystick_button_LT.publish(True);
			
		}else{
			pub_joystick_button_LT.publish(False);
		}
		
		if (msg->axes[5]<1)
		{
			pub_joystick_button_RT.publish(True);
			
		}else{
			pub_joystick_button_RT.publish(False);

		}
}

//starting joy

std::string joy_sub_str, cmd_vel_str;

int main (int argc, char** argv)
{
	ros::init(argc,argv,"joystick_core");
	
	ros::NodeHandle n("~");
	//read params of launch file
	n.param<std::string>("joy_sub", joy_sub_str, "/joy"); 
	n.param<std::string>("cmd_vel", cmd_vel_str, "/cmd_vel_joy"); 
	//sett publishers for joystick buttons for toggle states...
	pub_joystick_button_A = n.advertise<std_msgs::Bool>("button_A",100);
	pub_joystick_button_B = n.advertise<std_msgs::Bool>("button_B",100);
	pub_joystick_button_X = n.advertise<std_msgs::Bool>("button_X",100);
	pub_joystick_button_Y = n.advertise<std_msgs::Bool>("button_Y",100);
	pub_joystick_button_LB = n.advertise<std_msgs::Bool>("button_LB",100);
	pub_joystick_button_RB = n.advertise<std_msgs::Bool>("button_RB",100);
	pub_joystick_button_LT = n.advertise<std_msgs::Bool>("button_LT",100);
	pub_joystick_button_RT = n.advertise<std_msgs::Bool>("button_RT",100);
	
	//set publisher for cmd_vel with baud rate of 150
	pub_joystick_cmd_vel = n.advertise<geometry_msgs::TwistStamped>(cmd_vel_str.c_str(),150);	
	ros::Subscriber joy_sub= n.subscribe(joy_sub_str.c_str(),1,Joystick_callback);
		
	//take care that node is spinning...
	ros::spin();	
	
return 0;

}

