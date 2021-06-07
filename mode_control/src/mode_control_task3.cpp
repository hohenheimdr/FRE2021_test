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
	int pattern;//start pattern =1 for first turn left, 2 for first turn right
	
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
		if(pattern==1)
			ROS_INFO("next turn left");
		
		if(pattern==2)
			ROS_INFO("next turn right");
		
		if(mode.data!=USER_INPUT)
		{
		//ROS_INFO("check_mode");		
			//now you are allowed to change the mode :)
			switch(mode.data)
			{
				case 1:
				 ROS_INFO("obstacle=turn around");
					//change mode as soon as headland at the back was detected
					if(headland_detected_back)
						{
							sleep(1.0);
							last_mode=mode;
							new_mode.data=HEADLAND_TURN;
							new_mode.header.stamp=ros::Time::now();
							mode_pub.publish(new_mode);
							//turn_right backwards			
						}
				    break;
				case 2:
					ROS_INFO("row_navigation");
						//case the navigation is right now in row_navigation... the next possible case could be headland or obstacle
						 if(obstacle_detected)
						 {
							 last_mode=mode;
							//change to headland navigation
							 new_mode.data=HEADLAND_TURN;
							 new_mode.header.stamp=ros::Time::now();
							 mode_pub.publish(new_mode);
							 //obstacle... move backwards...or do a turnaround right now do a turn around and 
							 //toggle last turn
							 system("roslaunch fieldrobot_event2017 start_pattern_turn.launch");
							 if(pattern==1)
								pattern==2;
							 if(pattern==2)
								pattern==1;
				
							 last_mode=mode;
							//change to headland navigation
							new_mode.data=ROW_NAVIGATION;
							new_mode.header.stamp=ros::Time::now();
							mode_pub.publish(new_mode);
							 
						 } 
						 
						 if(headland_detected)
						{
							last_mode=mode;
							//change to headland navigation
							new_mode.data=HEADLAND_TURN;
							new_mode.header.stamp=ros::Time::now();
							mode_pub.publish(new_mode);
							//turn depending of the defined pattern for the headland goal...
							switch (pattern)
							{
								case 1:
									ROS_INFO("start 1L ");
									system("roslaunch fieldrobot_event2017 start_pattern_1L.launch");
									//move to next pattern every pattern can just be called once...
									pattern=2;
								break;
								case 2:
									ROS_INFO("start 1R ");
									system("roslaunch fieldrobot_event2017 start_pattern_1R.launch");
									//move to next pattern every pattern can just be called once...
									pattern=1;
								break;
								default:
								break;
							}
								
							last_mode=mode;
							//change to headland navigation
							new_mode.data=ROW_NAVIGATION;
							new_mode.header.stamp=ros::Time::now();
							mode_pub.publish(new_mode);
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
						//in this task we always move forward... throug the rows...
							//looks like the headland navigation is over... run now change to row navigation again
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
	n.param<int>("start_pattern", s.pattern, 1);//1=1L 2=1R
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
