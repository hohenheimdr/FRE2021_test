#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/TransformStamped.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "pathParser.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <ros/subscriber.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>


/* Goal Manager optimized for the 3D navigation of robot pinocchio
 * autor: david
 * date: 01.06.2017
 * function: 	reads the inputfile provided in the launch file settings "waypoint_list" as string, 
 * 				This coordinates are ordered in rows like x,y,z,waittime
 * 				coordinates get represented in the "fixed_frame" Frame
 * 				when the file should wait at each goal point for a topic this could be defined also
 * 				in "wait_for_topic" und wait_topic_name as std_msgs::Bool
 * 				also its possible to add configure mode, means, that the goal can be defined by the 
 * 				configure slides in dynamic reconfiguration 
 * 
 * topics out:	PointStamped. 
 * 
 * dr	added relative goals
 * dr 	removed configuration manager to speed system up
 * dr	removed user goal option to keep system reliable and small
 * dr	added expetion handling for the case when no tranform was found
 * */

using namespace std;

class SimpleGoal
{
private:	

	int actual_wp;
	ros::Time starttime;
	ros::Duration goal_waittime; //time to wait at actual goalpoint
	bool reached_last_waypoint,	topic_status,last_topic_status,time_break;
	geometry_msgs::TransformStamped goal_trans;
	pathParser waypoints;
	nav_msgs::Path path;
	
	
	// public variables
public:
	string fixed_frame, rabbit_frame, robot_frame;
	tf::TransformListener li;
	tf::TransformBroadcaster br;
	bool wait_for_topic,relative_goals, allow_break_after_seconds;
	ros::Publisher pose_goal_pub, point_goal_pub, path_pub;
	geometry_msgs::PoseStamped pose_goal,usergoal, config_goal, robot_pose;
	geometry_msgs::Point point_goal;
	double break_after_seconds;
	
	//start with normal constructor
	SimpleGoal(pathParser waypoints)
	{
		starttime=ros::Time::now();
		robot_frame="base_link";
		rabbit_frame="rabbit";
		fixed_frame="odom";
	    //set variables local to object
		this->waypoints=waypoints;
		//counter for waypoint list
		actual_wp = 0;
		time_break=false;
		topic_status=false;
		wait_for_topic=false;	
		reached_last_waypoint=false;
		//read planed path for visualisation purpose
	}
	
	~SimpleGoal()
	{
	}
	
	void read_robot_pose()
	{
		ROS_INFO("read robot pose");
		//define zero position
		robot_pose.header.stamp=ros::Time::now();
		robot_pose.header.frame_id=robot_frame;
		robot_pose.pose.position.x=0;
		robot_pose.pose.position.y=0;
		robot_pose.pose.position.z=0;
		robot_pose.pose.orientation.x=0;
		robot_pose.pose.orientation.y=0;
		robot_pose.pose.orientation.z=0;
		robot_pose.pose.orientation.w=1;
		tf::StampedTransform transform;
		try{
			//reading the actual robot pose relative to fixed frame
			li.waitForTransform(fixed_frame,robot_frame,robot_pose.header.stamp,ros::Duration(3.0));	//3.0	
			li.lookupTransform(fixed_frame,robot_frame,ros::Time(0), transform);	
			li.transformPose(fixed_frame,ros::Time(0),robot_pose,robot_frame,robot_pose);
			
			ROS_INFO("robot pose: x:%f,y:%f,z:%f",robot_pose.pose.position.x,robot_pose.pose.position.y,robot_pose.pose.position.z);
		
		}catch(...)
		{
			ROS_INFO("error getting robot transform");
		}
	}
	

	void goal_creater_loop()
	{		
		if(relative_goals)
		{
			read_robot_pose();
		}
		//run this loop until last waypoint was reached
		while(!reached_last_waypoint && !time_break)
		{
			get_next_wp();
			
			//if status should wait for topic...move in this loop
			if(wait_for_topic)
			{
				//wait until topic status is true or configure was received
				while(!topic_status && !time_break && !reached_last_waypoint)
				{
					if (allow_break_after_seconds)
					{
						//in this case we need to check the time 
						double time_duration=(ros::Time::now()-starttime).toSec();
						
						if(time_duration>break_after_seconds)
						{
							ROS_INFO("stoped goal point");
							time_break=true;
						//break;
						}
					}
									
					//read all callbacks since last call asnd stay in this loop until topic 
					ros::spinOnce();
					publish_goals();
					
				}
				//reset topic status!
				topic_status=false;
			}
			
			//just write goal constant if no configure was received!
						
			//if everything goes normal, we end here...
			publish_goals();	
			ros::spinOnce();
			ROS_INFO("sleeping now for %f seconds",goal_waittime.toSec());
			goal_waittime.sleep();
			//read all callbacks since last call bevore continue
			ros::spinOnce();
				
		}//end while loop	
	}
	
	void readPath()
	{
		ROS_INFO("read Path");
		//check what this part is doing here! (do I need it?)
		path.header.frame_id=fixed_frame;
		path.header.stamp=ros::Time::now();
		
		for (int i=0;i<waypoints.path.size();i++)
		{
			//ROS_INFO("create goalpath");
			geometry_msgs::PoseStamped point;
			point.header.stamp=ros::Time::now();
			point.header.frame_id=fixed_frame;
			point=waypoints.path[i];
			path.poses.push_back(point);
		}	
	}
	
		
	void publish_goals()
	{			
			//transform goal, when frame is not fixed frame
			pose_goal.header.stamp=ros::Time::now();
			PublishTransform();	
			//publish the path you shoud go to...
			publish_Path();	
			//publish goal as point 
			point_goal_pub.publish(pose_goal.pose.position);
			//publish goal as pose
			pose_goal_pub.publish(pose_goal);		
	}
	
	bool pose_equal(geometry_msgs::PoseStamped a,geometry_msgs::PoseStamped b)
	{ //return true if input a and b are the same
		if (a.pose.position.x==b.pose.position.x && a.pose.position.y==b.pose.position.y && a.pose.position.z==b.pose.position.z)
		{
			if (a.pose.orientation.x==b.pose.orientation.x && a.pose.orientation.y==b.pose.orientation.y && a.pose.orientation.z==b.pose.orientation.z)
			return true;
		}
			return false;
	}
		
	void publish_Path()
	{
		//publish the received path
		path.header.stamp=ros::Time::now();
		path.header.frame_id=fixed_frame;
		path_pub.publish(path);
	}
	
	void get_next_wp()
	{
		int number_left=waypoints.path.size()-actual_wp;
		ROS_INFO("get next waypoint! %i points in list",number_left);
		//check if actual wp is the last waypoint...
		if(actual_wp<waypoints.path.size())
		{
			pose_goal=waypoints.path[actual_wp];
			pose_goal.header.frame_id=fixed_frame;//fixed_frame is the reference frame!
			//read time to wait what was provided from the goal list
			goal_waittime=ros::Duration(pose_goal.header.stamp.toSec());
			pose_goal.header.stamp=ros::Time::now();
			actual_wp++;
			
			if(relative_goals)
				{
					try{
						PublishTransform();
						pose_goal.header.frame_id=robot_frame+"_old";
						tf::StampedTransform transform;
						//need to transform the goals to the robot starting position...
						ROS_INFO("adding a relative transform to goal!");
						//exception to get rid of transformation error:
						while(sqrt(pose_goal.pose.position.x*pose_goal.pose.position.x+pose_goal.pose.position.y*pose_goal.pose.position.y)>5.0)
						{
							li.waitForTransform(fixed_frame,robot_frame+"_old",ros::Time(0),ros::Duration(3.0)); //3.0		
							li.lookupTransform(fixed_frame,robot_frame+"_old",ros::Time(0), transform);	
							li.transformPose(fixed_frame,ros::Time(0),pose_goal,robot_frame+"_old",pose_goal);
						}
					}catch(...)
					{
						ROS_INFO("error trying relative goal transform");
					}
				
					ROS_INFO("goal pose: x:%f,y:%f,z:%f",pose_goal.pose.position.x,pose_goal.pose.position.y,robot_pose.pose.position.z);
				}
			
		} else
		{
			ROS_INFO("reached last waypoint!");
			reached_last_waypoint=true;
			actual_wp++;
		}
	}
		
	void check_topic(const std_msgs::Bool::ConstPtr& msg)
	{
		//do some error handling just change topic status if actual message is new! whould need change to bool_stamp...later!
		//just overwrite if the status was changed from outside!
		if (last_topic_status!=msg->data)
		topic_status=msg->data;
		//got ready status from topic
		ROS_INFO("got new status");	
		last_topic_status=topic_status;
	}
	
	void PublishTransform()
	{
			goal_trans.header.stamp=ros::Time::now();
			goal_trans.header.frame_id =pose_goal.header.frame_id;
			goal_trans.child_frame_id =rabbit_frame;
			goal_trans.transform.translation.x = pose_goal.pose.position.x;
			goal_trans.transform.translation.y = pose_goal.pose.position.y;
			goal_trans.transform.translation.z = pose_goal.pose.position.z;
			goal_trans.transform.rotation = pose_goal.pose.orientation;
			br.sendTransform(goal_trans);
			
			if(relative_goals)
			{
				//publish start transform
				goal_trans.header.stamp=ros::Time::now();
				goal_trans.header.frame_id =fixed_frame;
				goal_trans.child_frame_id =robot_frame+"_old";
				goal_trans.transform.translation.x = robot_pose.pose.position.x;
				goal_trans.transform.translation.y = robot_pose.pose.position.y;
				goal_trans.transform.translation.z = robot_pose.pose.position.z;
				goal_trans.transform.rotation = robot_pose.pose.orientation;
				br.sendTransform(goal_trans);
			}			
	}
	
		
	void TransformGoal()
	{	
		try{
				
				//check if transform is neccessary
				if(pose_goal.header.frame_id!=fixed_frame)
				{
					ROS_INFO("transforming goal now");
					tf::StampedTransform transform;
					//ros:Time(0) provides the last available transform!
					li.waitForTransform(fixed_frame,pose_goal.header.frame_id,ros::Time(0),ros::Duration(3.0));		//3.0
					li.lookupTransform(fixed_frame,pose_goal.header.frame_id,ros::Time(0), transform);	
					li.transformPose(fixed_frame,ros::Time(0),pose_goal,pose_goal.header.frame_id,pose_goal);
					//li.transformPoint(fixed_frame,ros::Time(0),
					//tf::Vector3 position=transform.getOrigin();
					//tf::Quaternion a=transform.getRotation();
					//tf::Quaternion b(pose_goal.pose.orientation.x,pose_goal.pose.orientation.y,pose_goal.pose.orientation.z,pose_goal.pose.orientation.w);
					//tf::Quaternion c=a*b.inverse();
					//pose_goal.pose.orientation.x=c[0];
					//pose_goal.pose.orientation.y=c[1];
					//pose_goal.pose.orientation.z=-c[2];
					//pose_goal.pose.orientation.w=c[3];
					
					//tf::Vector3 position=transform.transformPoint(pose_goal.pose.position.x,pose_goal.pose.position.y,pose_goal.pose.position.z);
					//pose_goal.pose.position.x=pose_goal.pose.position.x;//position[0];//+pose_goal.pose.position.x;
					//pose_goal.pose.position.y=pose_goal.pose.position.y;//position[1];//+pose_goal.pose.position.y;
					//pose_goal.pose.position.z=pose_goal.pose.position.z;//position[2];//+pose_goal.pose.position.z;
					
					pose_goal.header.frame_id=fixed_frame;
					
				}
			}
			catch(tf::TransformException ex)
			{
				ROS_INFO("error with transform from %s to %s",robot_frame.c_str(),fixed_frame.c_str());				
			}		
	}
			
};
//--------------------main-----------------------------------------------------

int main(int argc, char** argv){
  ros::init(argc, argv, "goal_creater");

ros::NodeHandle n("~");

std::string point_str,pose_str,wp_list_str,path_pub_str, usergoal_str,topic_name_str;
double m_distance;
bool slow_down;

	  n.param<std::string>("next_pose", pose_str, "/goal_pose");
	  n.param<std::string>("next_point", point_str, "/goal_point");  
	  n.param<std::string>("waypoint_list", wp_list_str, "waypoint.txt"); 
	  n.param<std::string>("path_publisher", path_pub_str, "/goalpath");
	  n.param<std::string>("wait_topic_name",topic_name_str, "wait");
	  
	  //read actual points
	  pathParser waypoints(wp_list_str.c_str());
	  //create object for waypoint management  
	  SimpleGoal g(waypoints);
	  n.param<std::string>("robot_frame",g.robot_frame, "base_link");
	  n.param<std::string>("rabbit_frame",g.rabbit_frame, "rabbit");
	  n.param<std::string>("fixed_frame",g.fixed_frame, "odom");
	  
	  n.param<bool>("wait_for_topic", g.wait_for_topic,false);
	  //need to set to false when GNSS coordinates get used
	  n.param<bool>("relative_goals", g.relative_goals,false); 
	   n.param<bool>("allow_break_after_seconds", g.allow_break_after_seconds,false); 
	  n.param<double>("break_after_seconds",g.break_after_seconds, 0);
	  
     //publishers:
      g.point_goal_pub=n.advertise<geometry_msgs::Point>(point_str.c_str(), 1);
      g.pose_goal_pub=n.advertise<geometry_msgs::PoseStamped>(pose_str.c_str(), 1);
	  g.path_pub=n.advertise<nav_msgs::Path>(path_pub_str.c_str(), 1);
	  //subscribers: 
      ros::Subscriber sub_topic= n.subscribe(topic_name_str,15, &SimpleGoal::check_topic, &g);
	  //read path and than start goal creater loop
	  g.readPath();
	  g.goal_creater_loop();
	  
	 //stop program after last goal reached.
	 
}
