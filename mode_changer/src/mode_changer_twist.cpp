#include "ros/ros.h"

#include "message_filters/subscriber.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <msgs/IntStamped.h>

#include <std_msgs/Int64.h>

//reads the signals from joystick topics and or another subscriber
//to change the modes for defining the motorcontroller command topic
//changed to Twist output for gazebo jackal 6.6.21




class Sender_Node
{
	
	#define USER_INPUT 0;	
	#define OBSTACLE 1;
	#define ROW_NAVIGATION 2;
	#define HEADLAND_TURN 3;
	#define STATIC 4;
	
private:
geometry_msgs::TwistStamped cmd_vel_row,cmd_vel_laser,cmd_vel_point,cmd_vel_joy,cmd_vel_static,cmd_vel_obstacle;
bool row_button, stop_button, action_button_1,action_button_2;
	//internal variable to save the actual mode
	msgs::IntStamped mode;
	int last_mode,count_points;
	ros::Duration time_duration;
	geometry_msgs::PoseStamped goal_last;

public:
	// public variables
	ros::Publisher pub_cmd_vel, headland_reset, mode_pub;
	double duration,deadzone;
	

	Sender_Node()
	{
		//set start mode to user input...
		mode.data=USER_INPUT;
		last_mode=USER_INPUT;
		mode.header.stamp=ros::Time::now();	
		count_points=0; 		
	}
	
	~Sender_Node()
	{
	}
	
	//-------continuous publisher called by the node to garante a constant output---------------
	void PublishSpeed(const ros::TimerEvent& e)
	{	
		ros::Duration time_dif_joy=(ros::Time::now()-cmd_vel_joy.header.stamp);
		ros::Duration time_dif_obstacle=(ros::Time::now()-cmd_vel_obstacle.header.stamp);
		ros::Duration time_dif_row=(ros::Time::now()-cmd_vel_row.header.stamp);
		ros::Duration time_dif_static=(ros::Time::now()-cmd_vel_static.header.stamp);
		ros::Duration time_dif_point=(ros::Time::now()-cmd_vel_point.header.stamp);
			switch (mode.data)
			{
				//user input
				case 0:
					//reset time
					if(time_dif_joy.toSec()<=deadzone)
					pub_cmd_vel.publish(cmd_vel_joy.twist);
					ROS_INFO("user_input");
					last_mode=USER_INPUT;
				break;
				//obstacle
				case 1:
					if(time_dif_obstacle.toSec()<=deadzone)
					pub_cmd_vel.publish(cmd_vel_obstacle.twist);
					ROS_INFO("obstacle_navigation");
					last_mode=OBSTACLE;
				break;
				//row_navigation
				case 2:
					if(time_dif_row.toSec()<=deadzone)
					pub_cmd_vel.publish(cmd_vel_row.twist);
					ROS_INFO("row_navigation");
					last_mode=ROW_NAVIGATION;
				break;
				//headland_ turning
				case 3:
					if(time_dif_point.toSec()<=deadzone)	
					pub_cmd_vel.publish(cmd_vel_point.twist);	
					ROS_INFO("headland_navigation");
					last_mode=HEADLAND_TURN;
			
				break;
				//static_cmd
				case 4:
					if(time_dif_static.toSec()<=deadzone)	
					pub_cmd_vel.publish(cmd_vel_static.twist);
					ROS_INFO("static_navigation");
					last_mode=STATIC;
				break;
				
				default:
				ROS_INFO("test_mode");
			}
			
			mode_pub.publish(mode);
	
	}
	//------subscribers----------
	void PointSpeed(const geometry_msgs::Twist::ConstPtr& msg)
	{
		 cmd_vel_point.twist=*msg;
		 cmd_vel_point.header.stamp=ros::Time::now();
		 
	}
	void RowSpeed(const geometry_msgs::Twist::ConstPtr& msg)
	{	
		 cmd_vel_row.header.stamp=ros::Time::now();
		 cmd_vel_row.twist=*msg;
		 
		 
	}
	void ObstacleSpeed(const geometry_msgs::Twist::ConstPtr& msg)
	{
		 cmd_vel_obstacle.twist=*msg;
		 cmd_vel_obstacle.header.stamp=ros::Time::now();
		 
	}
	
	void StaticSpeed(const geometry_msgs::Twist::ConstPtr& msg)
	{
		
		 cmd_vel_static.twist=*msg;
		 cmd_vel_static.header.stamp=ros::Time::now();	
	}
	
	
	void JoystickSpeed(const geometry_msgs::TwistStamped::ConstPtr& msg)
	{
		//if userinput, then directly forward the speed to the output...
		cmd_vel_joy=*msg;
		//cmd_vel_joy.header.stamp=ros::Time::now();
			if (cmd_vel_joy.twist.linear.x!=0 || cmd_vel_joy.twist.angular.z!=0)
			{
				mode.data=USER_INPUT;
				mode.header.stamp=ros::Time::now();	 
				pub_cmd_vel.publish(cmd_vel_joy.twist);
			}
	}
	
	void JoystickInput(const sensor_msgs::Joy::ConstPtr& msg)
	{
		sleep(0.1);
		//if userinput, then directly forward the speed to the output...
		if (!row_button &&  !stop_button && !action_button_1 && !action_button_2)
		{	
				mode.data=USER_INPUT;
				mode.header.stamp=ros::Time::now();	 
		}
	}
	
	//button subscribers
	void Row_button(const std_msgs::Bool::ConstPtr& msg)
	{
		row_button=msg->data; 
		//change mode... for the first...
		mode.data=ROW_NAVIGATION;
		mode.header.stamp=ros::Time::now();	
	}
	
	void Stop_Button(const std_msgs::Bool::ConstPtr& msg)
	{
		stop_button=msg->data;
		mode.data=OBSTACLE;
		mode.header.stamp=ros::Time::now();	
	}
	
	void Action_Button_1(const std_msgs::Bool::ConstPtr& msg)
	{
		action_button_1=msg->data;
		mode.data=STATIC;
		mode.header.stamp=ros::Time::now();	
	}
	
	void Action_Button_2(const std_msgs::Bool::ConstPtr& msg)
	{
		action_button_2=msg->data;
		mode.data=HEADLAND_TURN;
		mode.header.stamp=ros::Time::now();	
	}
	
	void Mode_Status(const msgs::IntStamped::ConstPtr& msg)
	{
		//check if last mode was not user_mode....nger ago than frequency
		//if mode not USER_MODE...no chance for programe to change the mode :)
		//check if there is the change of row to headland navigation....
		if(mode.data!=0)
		{			
			mode.data=msg->data;
			mode.header.stamp=ros::Time::now();		
		}		
	}
	
	double abs_d(double t)
	{
		return sqrt(t*t);
	}

};
//-------------------------------------------------------------------------


int main (int argc, char** argv)
{
	std::string cmd_vel_pub_str,cmd_vel_joy_str,cmd_vel_point_str,cmd_vel_static_str,cmd_vel_row_str,cmd_vel_obstacle_str;
	std::string row_button_str,stop_button_str,action_button_1_str,action_button_2_str,odom_reset_str,distance_str;
	std::string obstacle_front_str,obstacle_front_left_str,obstacle_front_right_str,joy_sub_str;
	std::string obstacle_back_str,obstacle_back_left_str,obstacle_back_right_str,mode_sub_str, mode_pub_str;

	ros::init(argc,argv,"sender_controller");
	ros::NodeHandle n("~");
	
	//read params of launch file
	
	if(!n.hasParam("cmd_vel_joy")) ROS_WARN("Used default parameter for  cmd_vel_joy");
	n.param<std::string>("joy_sub_left", cmd_vel_joy_str, "/cmd_vel_joy"); 
	if(!n.hasParam("cmd_vel_row")) ROS_WARN("Used default parameter for  cmd_vel_row");
	n.param<std::string>("cmd_vel_row", cmd_vel_row_str, "/cmd_vel_row"); 
	if(!n.hasParam("cmd_vel_point")) ROS_WARN("Used default parameter for  cmd_vel_point");
	n.param<std::string>("cmd_vel_headland", cmd_vel_point_str, "/cmd_vel_point"); 
	n.param<std::string>("cmd_vel_static", cmd_vel_static_str, "/cmd_vel_static");
	n.param<std::string>("cmd_vel_obstacle", cmd_vel_obstacle_str, "/cmd_vel_obstacle");  
	if(!n.hasParam("cmd_vel_pub")) ROS_WARN("Used default parameter for cmd_vel_pub");
	n.param<std::string>("cmd_vel_pub", cmd_vel_pub_str, "/cmd_vel"); 
	//not neccessary but useful :)
	n.param<std::string>("row_button", row_button_str, "/row_button"); 
	n.param<std::string>("obstacle_button", stop_button_str, "/stop_button"); 
	n.param<std::string>("static_button", action_button_1_str, "/action_button_1"); 
	n.param<std::string>("headland_button", action_button_2_str, "/action_button_2"); 
	n.param<std::string>("mode_sub", mode_sub_str, "/change_mode");
	n.param<std::string>("mode_pub", mode_pub_str, "/actual_mode");
	n.param<std::string>("joy_sub", joy_sub_str, "/joy");
	n.param<std::string>("odom_reset", odom_reset_str, "/odom_set");
	n.param<std::string>("goal_sub",distance_str, "/goal");
	  
	//subscribers for the error stop handling
		
	Sender_Node s;
	//in seconds...
	n.param<double>("mode_change_duration", s.duration, 0.5);
	n.param<double>("deadzone",s.deadzone,0.05);
	//speed cmd subscribers
	ros::Subscriber joy=n.subscribe(cmd_vel_joy_str,2,&Sender_Node::JoystickSpeed,&s);
	ros::Subscriber joy_in=n.subscribe(joy_sub_str,2,&Sender_Node::JoystickInput,&s);
	ros::Subscriber row=n.subscribe(cmd_vel_row_str,2,&Sender_Node::RowSpeed,&s);
	ros::Subscriber point=n.subscribe(cmd_vel_point_str,2,&Sender_Node::PointSpeed,&s);
	ros::Subscriber _static=n.subscribe(cmd_vel_static_str,2,&Sender_Node::StaticSpeed,&s);
	ros::Subscriber _obstacle=n.subscribe(cmd_vel_obstacle_str,2,&Sender_Node::ObstacleSpeed,&s);
	//joystick button subscribers
	ros::Subscriber a=n.subscribe(row_button_str,2,&Sender_Node::Row_button,&s);	
	ros::Subscriber b=n.subscribe(stop_button_str,2,&Sender_Node::Stop_Button,&s);
	ros::Subscriber x=n.subscribe(action_button_1_str,2,&Sender_Node::Action_Button_1,&s);
	ros::Subscriber y=n.subscribe(action_button_2_str,2,&Sender_Node::Action_Button_2,&s);
	//mode subscriber
	ros::Subscriber m=n.subscribe(mode_sub_str,2,&Sender_Node::Mode_Status,&s);
	//publishers
	s.pub_cmd_vel = n.advertise<geometry_msgs::Twist>(cmd_vel_pub_str.c_str(),3);
	s.mode_pub = n.advertise<msgs::IntStamped>(mode_pub_str.c_str(),3);
	//define the update rate by a seperate function...
	ros::Timer t;
	t = n.createTimer(ros::Duration(0.02), &Sender_Node::PublishSpeed,&s);
	
	ros::spin();
		
}
