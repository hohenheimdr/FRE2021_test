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
	bool line_detected,row_detected,movement_finished,obstacle_detected,last_turn_left;
	int USER_INPUT= 0;
	int OBSTACLE= 1;
	int ROW_NAVIGATION= 2;
	int HEADLAND_TURN= 3;
	int STATIC= 4;
	int pattern=1;
	int line_counter;
	std_msgs::Bool seeder_signal;

public:	
	double mode_duration;
	geometry_msgs::Twist cmd_motor;
	
	ros::Publisher mode_pub, motor_pub,seeder_pub;

	Mode_Control()
	{
		//set start mode to user input...
		mode.data=USER_INPUT;
		mode.header.stamp=ros::Time::now();	
		new_mode.data=USER_INPUT;
		new_mode.header.stamp=ros::Time::now();			
		last_mode=mode;
		last_turn_left=true;
		line_detected=false;
		row_detected=false;
		movement_finished=false;
		obstacle_detected=false;
		//variable for 
		line_counter=0;
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
				 ROS_INFO("obstacle=row_backwards..do nothing");
					//change mode as soon as headland at the back was detected
					
				    break;
				case 2:
					ROS_INFO("row_navigation");
						//case the navigation is right now in row_navigation... the next possible case could be headland or obstacle
						 if(obstacle_detected)
						 {
							 //obstacle... move backwards...
							 last_mode=mode;
							//change to headland navigation
							new_mode.data=OBSTACLE;
							new_mode.header.stamp=ros::Time::now();
							mode_pub.publish(new_mode);
						 }
							
							//case we followed the row and detected a line...
							
							line_counter++;
							
							switch (line_counter)
							{
								case 1: 
								
								ROS_INFO("detected seeder line");
								//Move_Motor_Up();
								
									last_mode=mode;
									new_mode.data=HEADLAND_TURN;
									new_mode.header.stamp=ros::Time::now();
									mode_pub.publish(new_mode);
									//take care that vehicle stops...and start directly the launch file for moving to the seeder position
									ROS_INFO("move to seeder");
									system("roslaunch fieldrobot_event2016 move_to_seeder.launch");
									//now we need to move forward until seeder is under 
									ROS_INFO("send seeder command");
									seeder_signal.data=true;
									for( int i=0;i<1000;i++)
									{
										seeder_pub.publish(seeder_signal);
									}
									sleep(1);
									system("roslaunch fieldrobot_event2016 move_in_field.launch");
									system("roslaunch fieldrobot_event2016 start_pattern_1R");
									system("roslaunch fieldrobot_event2016 move_in_field.launch");
									
									
									//sleep(10);
									//seeding accomplished
									ROS_INFO("seeding accomplished");
									//move again to row navigation
									last_mode=mode;
									new_mode.data=OBSTACLE;
									new_mode.header.stamp=ros::Time::now();
									mode_pub.publish(new_mode);
								break;
								
								case 2: 
								ROS_INFO("move to seeding area");
								//we reached the seeding area....
								system("roslaunch fieldrobot_event2016 move_to_field.launch");
								Move_Motor_Down();
								//change to row navigation
								ROS_INFO("seeder is down now");
								//move again to row navigation
									last_mode=mode;
									new_mode.data=ROW_NAVIGATION;
									new_mode.header.stamp=ros::Time::now();
									mode_pub.publish(new_mode);
								break;
								
								case 3: 
								ROS_INFO("detected headland line");
									last_mode=mode;
									new_mode.data=HEADLAND_TURN;
									new_mode.header.stamp=ros::Time::now();
									mode_pub.publish(new_mode);
									system("roslaunch fieldrobot_event2016 move_in_field.launch");
									Move_Motor_Up();
									
									ROS_INFO("seeder is up now");
									system("roslaunch fieldrobot_event2016 start_pattern_1R");
									ROS_INFO("turn is over_now");
									system("roslaunch fieldrobot_event2016 move_in_field.launch");
									Move_Motor_Down();
									ROS_INFO("follow line");
									last_mode=mode;
									new_mode.data=ROW_NAVIGATION;
									new_mode.header.stamp=ros::Time::now();
									mode_pub.publish(new_mode);
								break;
								
								case 4: 
									ROS_INFO("detected end of seeding line");
									last_mode=mode;
									new_mode.data=HEADLAND_TURN;
									new_mode.header.stamp=ros::Time::now();
									mode_pub.publish(new_mode);
									//system("roslaunch fieldrobot_event2016 move_seeder.launch");
									Move_Motor_Up();
									ROS_INFO("seeder is up now");
									system("roslaunch fieldrobot_event2016 move_to_field.launch");
									last_mode=mode;
									new_mode.data=ROW_NAVIGATION;
									new_mode.header.stamp=ros::Time::now();
									mode_pub.publish(new_mode);
								
								break;
								case 5: 
								ROS_INFO("detected seeder line");
									last_mode=mode;
									new_mode.data=OBSTACLE;
									new_mode.header.stamp=ros::Time::now();
									mode_pub.publish(new_mode);
								break;
								case 6: 
								ROS_INFO("detected seeder line");
								break;
								default:
								break;	
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
						
			line_detected=false;
			row_detected=false;
			//movement_finished=false;
			obstacle_detected=false;
			
		}
	}
	
	void Move_Motor_Up()
	{
		geometry_msgs::Twist cmd;
		cmd.linear.x=-1.0;
		for( int i=0;i++;i<100)
		{
			ROS_INFO("move seeder up");
			motor_pub.publish(cmd);
			sleep(1);
		}
	}
	
	void Move_Motor_Down()
	{
		geometry_msgs::Twist cmd;
		cmd.linear.x=1.0;
		for( int i=0;i++;i<100)
		{
			ROS_INFO("move seeder down");
			motor_pub.publish(cmd);
			sleep(0.1);
		}
	}
	
	//------subscribers----------
	void ActualMode(const msgs::IntStamped::ConstPtr& msg)
	{
		//ROS_INFO("checked mode");
		 mode=*msg;	 
	}
	
	void Line_Detected(const std_msgs::Bool::ConstPtr& msg)
	{
		//ROS_INFO("checked headland front");
		line_detected=msg->data;	
	}
	
	void Row_Detected(const std_msgs::Bool::ConstPtr& msg)
	{
		//ROS_INFO("checked row");
		row_detected=msg->data;
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
	std::string headland_str,obstacle_str,movement_str,mode_pub_str,headland_back_str,seeder_pub_str,motor_pub_str;
	double frequency;
	
	n.param<std::string>("mode_sub", mode_sub_str, "/actual_mode");
	n.param<std::string>("mode_pub", mode_pub_str, "/mode_pub");
	n.param<std::string>("line_detected", headland_str, "/line_detected");
	n.param<std::string>("movement_finished", movement_str, "/movement_finished"); 
	n.param<double>("frequency", frequency, 5); 
	n.param<std::string>("seeder_button", seeder_pub_str, "/joy/button_LT");
	n.param<std::string>("motor_seeder_cmd", motor_pub_str, "/joy/button_LB");
			
	Mode_Control s;
	//in seconds...
	n.param<double>("mode_change_duration", s.mode_duration, 0.5);
	//mode subscriber
	ros::Subscriber m=n.subscribe(mode_sub_str,10,&Mode_Control::ActualMode,&s);
	ros::Subscriber m2=n.subscribe(headland_str,10,&Mode_Control::Line_Detected,&s);
	
	s.seeder_pub = n.advertise<std_msgs::Bool>(seeder_pub_str.c_str(),10);
	s.mode_pub = n.advertise<msgs::IntStamped>(mode_pub_str.c_str(),10);
	s.motor_pub = n.advertise<geometry_msgs::Twist>(motor_pub_str.c_str(),10);

	//define the update rate by a seperate function...
	ros::Timer t;
	t = n.createTimer(ros::Duration(1.0/frequency), &Mode_Control::CheckMode,&s);
	
	ros::spin();
		
}
