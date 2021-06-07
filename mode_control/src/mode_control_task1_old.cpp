#include "ros/ros.h"

#include "message_filters/subscriber.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <msgs/IntStamped.h>

#include <std_msgs/Int64.h>

//reads the actual mode and change and actuate everything as neccessary for task 1 
//optimized for task 1 fieldrobot event 2016
//author: David Reiser
//date: 2.6.16

//subscribes to actual mode and all other informations like headland detected, obstacle detected, etc,
//movement finished...
//and change the mode depending on the need


class Mode_Control
{
		
private:
	msgs::IntStamped mode, last_mode,new_mode;
	bool headland_detected,row_detected,movement_finished,obstacle_detected,last_turn_left,headland_detected_back;
	int USER_INPUT= 0;
	int OBSTACLE= 1;
	int ROW_NAVIGATION= 2;
	int HEADLAND_TURN= 3;
	int STATIC= 4;

public:	
	double mode_duration;
	
	ros::Publisher mode_pub;

	Mode_Control()
	{
		//set start mode to user input...
		mode.data=USER_INPUT;
		mode.header.stamp=ros::Time::now();	
		new_mode.data=USER_INPUT;
		new_mode.header.stamp=ros::Time::now();			
		last_mode=mode;
		last_turn_left=true;
		headland_detected=false;
		row_detected=false;
		movement_finished=false;
		obstacle_detected=false;
	}
	
	~Mode_Control()
	{
	}
	
	//-------continuous publisher called by the node to garante a constant output---------------
	void CheckMode(const ros::TimerEvent& e)
	{	
		if(mode.data!=USER_INPUT)
		{
		//ROS_INFO("check_mode");		
			//now you are allowed to change the mode :)
			switch(mode.data)
			{
				case 1:
				 ROS_INFO("obstacle=row_backwards");
					//change mode as soon as headland at the back was detected
					if(headland_detected_back)
						{
							last_mode=mode;
							new_mode.data=HEADLAND_TURN;
							new_mode.header.stamp=ros::Time::now();
							mode_pub.publish(new_mode);
							//turn_right backwards
							ROS_INFO("start backwards right");
							//need change soon...
							system("roslaunch fieldrobot_event2016 start_goalmanager_headland_right_back.launch");
						}
				    break;
				case 2:
					ROS_INFO("row_navigation");
						//case the navigation is right now in row_navigation... the next possible case could be headland or obstacle
						 if(headland_detected)
						{
							last_mode=mode;
							//change to headland navigation
							new_mode.data=HEADLAND_TURN;
							new_mode.header.stamp=ros::Time::now();
							mode_pub.publish(new_mode);
							//turn right
							ROS_INFO("start right turn forward ");
							system("roslaunch fieldrobot_event2016 start_goalmanager_headland_right.launch");
						}
					break;
				case 3:
					ROS_INFO("headland_navigation");
					//check if last point was reached... so wait if machine starts moving again
					if(movement_finished)
						{
						sleep(0.1);
						ROS_INFO("no movement...");
						}
					if(movement_finished)
					{
							//looks like the headland navigation is over... run now change to row navigation again
							if(last_mode.data==ROW_NAVIGATION)
								new_mode.data=OBSTACLE;
							if(last_mode.data==OBSTACLE)
								new_mode.data=ROW_NAVIGATION;
							new_mode.header.stamp=ros::Time::now();
							//save last mode;
							last_mode=mode;
							mode_pub.publish(new_mode);	
					}
					
					break;
				case 4:
					ROS_INFO("static_navigation");
							//leave it like it is...just for the case
					break;
				
				default:
				break;
			}
			
			
			headland_detected=false;
			headland_detected_back=false;
			row_detected=false;
			//movement_finished=false;
			obstacle_detected=false;
		}
	
	}
	//------subscribers----------
	void ActualMode(const msgs::IntStamped::ConstPtr& msg)
	{
		//ROS_INFO("checked mode");
		 mode=*msg;	 
	}
	
	void Headland_Detected(const std_msgs::Bool::ConstPtr& msg)
	{
		//ROS_INFO("checked headland front");
		headland_detected=msg->data;	
	}
	void Headland_Detected_Back(const std_msgs::Bool::ConstPtr& msg)
	{
		//ROS_INFO("checked headland back");
		headland_detected_back=msg->data;	
	}
	
	void Row_Detected(const std_msgs::Bool::ConstPtr& msg)
	{
		//ROS_INFO("checked row");
		row_detected=msg->data;
	}
	
	void Movement_Finished(const std_msgs::Bool::ConstPtr& msg)
	{
		//ROS_INFO("checked movement");
		movement_finished=msg->data;
	}
	void Obstacle_Detected(const std_msgs::Bool::ConstPtr& msg)
	{
		//ROS_INFO("checked obstacle");
		obstacle_detected=msg->data;
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
	std::string start_button_str,stop_button_str,action_button_1_str,action_button_2_str,odom_reset_str,distance_str;
	std::string obstacle_front_str,obstacle_front_left_str,obstacle_front_right_str,joy_sub_str;
	std::string obstacle_back_str,obstacle_back_left_str,obstacle_back_right_str,mode_sub_str;

	ros::init(argc,argv,"sender_controller");
	ros::NodeHandle n("~");
	
	//read params of launch file
	std::string headland_str,obstacle_str,movement_str,mode_pub_str,headland_back_str;
	double frequency;
	
	n.param<std::string>("mode_sub", mode_sub_str, "/actual_mode");
	n.param<std::string>("mode_pub", mode_pub_str, "/mode_pub");
	n.param<std::string>("headland_detected_front", headland_str, "/headland_detected");
	n.param<std::string>("headland_detected_back", headland_back_str, "/headland_detected");
	n.param<std::string>("obstacle_detected", obstacle_str, "/obstacle_detected");
	n.param<std::string>("movement_finished", movement_str, "/movement_finished"); 
	n.param<double>("frequency", frequency, 5); 
			
	Mode_Control s;
	//in seconds...
	n.param<double>("mode_change_duration", s.mode_duration, 0.5);
	//mode subscriber
	ros::Subscriber m=n.subscribe(mode_sub_str,10,&Mode_Control::ActualMode,&s);
	ros::Subscriber m2=n.subscribe(headland_str,10,&Mode_Control::Headland_Detected,&s);
	ros::Subscriber m3=n.subscribe(headland_back_str,10,&Mode_Control::Headland_Detected_Back,&s);
	ros::Subscriber m4=n.subscribe(obstacle_str,10,&Mode_Control::Obstacle_Detected,&s);
	ros::Subscriber m5=n.subscribe(movement_str,10,&Mode_Control::Movement_Finished,&s);
	
	s.mode_pub = n.advertise<msgs::IntStamped>(mode_pub_str.c_str(),10);
	//define the update rate by a seperate function...
	ros::Timer t;
	t = n.createTimer(ros::Duration(1.0/frequency), &Mode_Control::CheckMode,&s);
	
	ros::spin();
		
}
