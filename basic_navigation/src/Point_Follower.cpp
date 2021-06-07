/*
 * PointFollower.cpp
 *
 *  Created on: 12.10.15 
 *      Author: david
 * 
 * dr: some changes for the fieldrobot event 28.5.16
 *		- include PID
 * 		- include turn on spot
 * 		- include
 * 
 * dr 06.06.17 	changed code so that the point follower will not move backwards
 * 				as this is not neccessary with the pinocchio
 * dr 06.06.17	changed sampling speed to 0.05 check if slower suficient!
 * dr 07.06.17	cleaned up the whole structure
 * dr 10.06.127	changed angle_speed_min und speed_min from linear_offset and angular_offset 
 * 
 */


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <math.h>
#include <tf/transform_listener.h>


class PointFollower
{
	private:
		double distance, heading,last_heading,last_distance,rel_heading;
		double integral,last_heading_error, sampling_speed;
		tf::TransformListener tf_listen;
		geometry_msgs::Twist cmd_vel;
		tf::Vector3 xyz;
		ros::Time last_tf_time;

	public:
		std::string r_frame,v_frame, odom_frame;
		double thresh_min,dist_thresh,angle_thresh, max_integral;
		double angle_speed_min,speed_max,angle_speed_max,speed_min;
		double P,I,D,angle_react_thresh,start_moving_angle,time_offset;
		ros::Publisher cmd_vel_pub,goal_distance_pub, goal_reached_pub;
		bool use_PID_for_turn,allow_backwards;

		PointFollower ()
		{
			//set values to zero what need to stay zero first
			cmd_vel.linear.x=0;
			cmd_vel.angular.z=0;
			sampling_speed=0;
			
		}
		~PointFollower()
		{
			//stop program with zero command for the speed
			cmd_vel.angular.z=0;	
			cmd_vel.linear.x=0;
			cmd_vel_pub.publish(cmd_vel);
		}
		
		void spin(const ros::TimerEvent& e)
		{
			getDistance();
			createSpeed();
		}
		
		void getDistance()
		{
			try
			{
				tf::StampedTransform transformer;
				tf_listen.waitForTransform(v_frame,r_frame,ros::Time(0),ros::Duration(0.05));		
				tf_listen.lookupTransform(v_frame,r_frame,ros::Time(0), transformer);
				xyz = transformer.getOrigin();
				last_heading=heading;
				heading =atan2(xyz[1],xyz[0]);
					
				last_distance=distance;
				distance = sqrt(xyz[0]*xyz[0] + xyz[1]*xyz[1]);
				std_msgs::Float64 dist;
				dist.data=distance;
				//publish the distance as a topic
				goal_distance_pub.publish(dist);
				rel_heading=transformer.getRotation().getZ();
				sampling_speed=(ros::Time::now()-last_tf_time).toSec();
				ROS_INFO("Found rabbit at x=%.3f y=%.3f angle=%.3f dist=%.3f sampling_speed : %f",xyz[0],xyz[1],heading,distance, sampling_speed);
				//save time...
				last_tf_time=ros::Time::now();
			}
			catch (tf::TransformException& ex){
				ROS_DEBUG_THROTTLE(1,"tf: FAILED!");
				ROS_DEBUG_THROTTLE(1,"%s",ex.what());
				heading = 0;
				distance = 0;
				last_distance=0;
				sampling_speed=0;
				cmd_vel.angular.z=0;	
				cmd_vel.linear.x=0;
			}
		}
	
		void createSpeed()
		{
			double heading_error=heading;
			
			if(heading_error>1.57 && allow_backwards)
					heading_error=3.1415-heading_error;
				if(heading_error<-1.57 && allow_backwards)
					heading_error=-3.1415-heading_error; 
					
			//take care that all errors get a maximum of +/-1 as value		
			heading_error=double(heading_error/M_PI);
			
			//summ up the error for the integral value
			integral=(integral+heading_error);
			//check the max value of the integral and limit it!
			if (integral>max_integral)
				integral=max_integral;
			if(integral<-max_integral)
				integral=-max_integral;
			//define deviation change
			double deviation=(heading_error-last_heading_error);
			last_heading_error=heading_error;
			ROS_INFO("heading_error :%f, integral : %f, deviation : %f",heading_error, integral, deviation);
			
			//now first start to define the neccessary angular speed
			//check if the angle difference is higher than the threshold for the angle
			double new_angular_speed;
			if(abs_d(heading_error)>angle_react_thresh){	
					new_angular_speed=angle_speed_max*(P*heading_error+I*integral+D*deviation);
					//here just a normal P value will be used maybe problematic at small headings_errors
					if(!use_PID_for_turn)
						new_angular_speed=angle_speed_max*heading_error;
			}
			
			//check if the resulting speed is in the expected limits min/max speed
			if(new_angular_speed>angle_speed_max)
				new_angular_speed=angle_speed_max;
			if(new_angular_speed<-angle_speed_max)
				new_angular_speed=-angle_speed_max;
			if(new_angular_speed>0 && new_angular_speed<angle_speed_min)
				new_angular_speed=angle_speed_min;
			if(new_angular_speed<0 && new_angular_speed>-angle_speed_min)
				new_angular_speed=-angle_speed_min;
			//check if the threshold for the heading was reached--- this could maybe changed....need to see!
			
			if(abs_d(heading_error)<angle_thresh)
			{
				//as in this case the vehicle will start moving, we can accept values below angle_speed_min for turn
				//this helps to fine justate the position
				//new_angular_speed=0;
				new_angular_speed=angle_speed_max*(P*heading_error+I*integral+D*deviation);
			}
			//write back the angular speed to the cmd_vel
			cmd_vel.angular.z=new_angular_speed;
			
			//if heading error is small enough, and the distance to goal is not below threshold, start moving.
			double distance_error=abs_d(xyz[0]);
			
			if(abs_d(heading_error)<start_moving_angle && distance>dist_thresh)
			{
				if(distance_error>1)
				{
					distance_error=1;
					cmd_vel.linear.x=speed_max;
					
				}else{
					//define forward speed with heading error... as smaller heading error, as faster we could go
					//change later!
					//speed factor should be something between 0 and 1 ?
					double speed_factor=1;
					//if(heading_error==0)
						//speed_factor=start_moving_angle/abs_d(heading_error);
					cmd_vel.linear.x=speed_max*(distance_error*speed_factor);
					}		
			}
			//limit speed to speed max
			if(abs_d(cmd_vel.linear.x)>speed_max)
				cmd_vel.linear.x=speed_max;
			
			if(abs_d(cmd_vel.linear.x)<speed_min && cmd_vel.linear.x!=0)
				cmd_vel.linear.x=speed_min;
			//if we reached the dist_thresh, we do not need a speed_min any more.. maybe helps for fine justation
			if(distance<dist_thresh)
				cmd_vel.linear.x=0;
					
			if(xyz[0]<0 && allow_backwards){
				cmd_vel.linear.x=-abs_d(cmd_vel.linear.x);
				//case if we move backwards, we have to inverse the angle value
				cmd_vel.angular.z=-cmd_vel.angular.z;
			}	
				
			//define when the goal was reached.. in this case the next goal will set by the publisher
			std_msgs::Bool dummy;

			//check if speed is  in the acceptable limits

			//check if robot is turned right to the goal, if not, first turn to the right direction!
			
			//when timeout is longer than time_offset (in seconds), skip everything. 
			// this prevents the robot from running straight after signal was lost
			if(abs_d((ros::Time::now()-last_tf_time).toSec())>time_offset)
			{
				ROS_INFO("no goal...");
				sampling_speed=0;
				cmd_vel.linear.x=0;
				cmd_vel.linear.y=0;
				cmd_vel.linear.z=0;
				cmd_vel.angular.y=0;
				cmd_vel.angular.x=0;
				cmd_vel.angular.z=0;
				integral=0;
			}else if(abs_d(rel_heading)<angle_thresh && distance<dist_thresh)
				{	
					sampling_speed=0;
					cmd_vel.linear.x=0;
					cmd_vel.linear.y=0;
					cmd_vel.linear.z=0;
					cmd_vel.angular.y=0;
					cmd_vel.angular.x=0;
					cmd_vel.angular.z=0;
					integral=0;
					dummy.data=true;
					ROS_INFO("reached_goal!");
					goal_reached_pub.publish(dummy);
					//goal_reached_pub.publish(dummy);
					sleep(time_offset);
					
					//set the value back to false
				}else
				{
					dummy.data=false;
					goal_reached_pub.publish(dummy);
				}	
			

			//publish the actual value
			cmd_vel_pub.publish(cmd_vel);
			ROS_INFO("x=%f z=%f" , cmd_vel.linear.x,cmd_vel.angular.z);
		}
		
		double abs_d(double in){
			return sqrt(in*in);
		}
};


int main(int argc, char **argv) {

	//Initialize ros usage
	ros::init(argc, argv, "PointFollower");

	std::string topic_id, goal_id, goal_reached;

	//Create Nodehandlers
	ros::NodeHandle n("~");
	
	PointFollower pf;

	n.param<std::string>("rabbit_frame_id",pf.r_frame,"/rabbit");
	n.param<std::string>("vehicle_frame_id",pf.v_frame,"/base_link");
	n.param<std::string>("odom_frame_id",pf.odom_frame,"/odom");
	n.param<std::string>("cmd_vel_topic_id",topic_id,"/cmd_vel");
	n.param<std::string>("goal_distance",goal_id,"/goal_distance");
	n.param<std::string>("goal_reached",goal_reached,"/goal_reached");
	n.param<double>("angle_speed_min",pf.angle_speed_min,0.1);
	
	n.param<double>("speed_min",pf.speed_min,0.1);
	//distance for defining if goal was reached
	n.param<double>("dist_thresh",pf.dist_thresh,0.1);
	//difference acceptable for angle
	n.param<double>("angle_thresh",pf.angle_thresh,0.1);
	//just react on angle changes when this tresh was reached!
	n.param<double>("angle_react_thresh",pf.angle_react_thresh,0.0);
	n.param<double>("start_moving_angle",pf.start_moving_angle,0.75);
	n.param<double>("time_offset",pf.time_offset,0.1);
	n.param<double>("speed_max",pf.speed_max,0.1);
	n.param<double>("angle_speed_max",pf.angle_speed_max,0.1);
	n.param<bool>("use_PID_for_turn",pf.use_PID_for_turn,true);
	n.param<double>("P",pf.P,0.1);
	n.param<double>("I",pf.I,0.1);
	n.param<double>("D",pf.D,0.1);
	n.param<double>("max_integral",pf.max_integral,10);
	n.param<bool>("allow_backwards",pf.allow_backwards,true);
	
	pf.cmd_vel_pub = n.advertise<geometry_msgs::Twist>(topic_id.c_str(),10);
	pf.goal_distance_pub = n.advertise<std_msgs::Float64>(goal_id.c_str(),10);
	pf.goal_reached_pub = n.advertise<std_msgs::Bool>(goal_reached.c_str(),10);
	
	ros::Timer t = n.createTimer(ros::Duration(0.1),&PointFollower::spin,&pf);

	ros::spin();

}


