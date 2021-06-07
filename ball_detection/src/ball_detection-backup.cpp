#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <boost/thread/thread.hpp>
#include <boost/bind/bind.hpp>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <visualization_msgs/Marker.h>
#include <ctime>
#include <tf/transform_broadcaster.h>
#include "sensor_msgs/PointCloud2.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


// comments: 
//dr 	08.06.17	added output as pointcloud

//todos: make picture representation optional, best with launchfile option
//include output of std::bool when ball was detected
//include option to push the reference frame to a x+ level#
//output result picture as message



#define PI 3.14159265
 
using namespace cv;
using namespace std;

struct Ball_Data
{
	int id; //ID ist gleich ID von Marker
	int x;			//Coordinates
	int y;
	int r;
	time_t firstDetect;
	time_t lastDetect;
	
}; 

class Ball_detect
{
	private:
	
	cv_bridge::CvImagePtr cv_ptr,processed;	
		
	public:
	
	int H_min,H_max, S_min,S_max,V_min,V_max;
	int minKnownTime, maxLostTime, minRadius, maxRadius;
	int CalibAreaCornerX, CalibAreaHight, CalibAreaWidth;
	int ballcounter;
	
	time_t timestamp;
	
	
	double CamHight;
	double CamAoV; 		//Cam Angle of View
	int CamXMax;		// Maße des USB Cam Images mit dem Format 4:3
	int CamYMax;
	
	vector<Ball_Data> known_balls;
	
	ros::Publisher image1,image2;
	ros::Publisher marker_pub, cloud_pub;
	geometry_msgs::Point point;
	visualization_msgs::Marker points;
		

	Ball_detect() //constructor in Form einer Funktion
	{
		// Bereich für Parameter Einstellungen!!

		H_min=0;
		S_min=0;
		V_min=0;
		H_max=0;
		S_max=0;
		V_max=0;

		timestamp = time(0);
		minKnownTime = 2; //Time how long the Objekt should be known before publishing
		maxLostTime = 1; //Max Seconds a Ball was not detect;
		
		CalibAreaCornerX=270;
		CalibAreaHight=100;
		CalibAreaWidth=100;	
		
		CamHight = 0.68;
		CamAoV = 40.0;
		CamXMax=640;
		CamYMax=480;
		
		ballcounter=0;
		
		//Marker settings
		points.header.frame_id = "/Ball_detection";
		points.header.stamp = ros::Time::now();
		points.ns = "Detected_Ball";
		//points.type = visualization_msgs::Marker::POINTS;
		points.type = visualization_msgs::Marker::SPHERE;
		points.action = visualization_msgs::Marker::ADD;
		points.pose.orientation.w = 1.0;
		// POINTS markers use x and y scale for width/height respectively
		points.scale.x,points.scale.y,points.scale.z = 0.05;

		// Points are green
		points.color.g = 0.0f;	//g= green
		points.color.r = 0.5f;	//r= green
		points.color.b = 0.0f;	//g= green
		points.color.a = 1.0;  // a = visible, 0 => transparent
		points.lifetime = ros::Duration(); //The lifetime field specifies how long this marker should stick around before being automatically deleted. A value of ros::Duration() means never to auto-delete. 
		points.id = 0; 

		namedWindow("Processed");
		namedWindow("Color_scale");
		namedWindow("Result");
		namedWindow("Result2");
 		/// Create Trackbars for Manual HSV Filter Setup
 		createTrackbar("minRadius","Color_scale",&minRadius,100,NULL);
 		createTrackbar("maxRadius","Color_scale",&maxRadius,200,NULL);
 		createTrackbar("CalibAreaCornerX","Color_scale",&CalibAreaCornerX,CamXMax,NULL);
 		createTrackbar("CalibAreaHight","Color_scale",&CalibAreaHight,CamYMax,NULL);
 		createTrackbar("CalibAreaWidth","Color_scale",&CalibAreaWidth,CamXMax,NULL);
 	}
		
	~Ball_detect() //destructor
	{
		destroyWindow("Processed");
		destroyWindow("Color_scale");
		destroyWindow("Result");
		destroyWindow("Result2");
	} 
	
	
	void readImage(const sensor_msgs::Image::ConstPtr &msg)
	{
		//write callback topic  to private opencv variable
		try
		{ 
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
			processed = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);		
		}
			
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
		
		timestamp = time(0);

		points.header.frame_id = msg->header.frame_id;
		
		processed->image=processImage(cv_ptr->image);			

		processed->image=processGaussian (processed->image);
		image2.publish(processed->toImageMsg());
		
		cv_ptr->image=processHugh(processed->image,cv_ptr->image);
		image1.publish(cv_ptr->toImageMsg());	

		publishKnownBalls();

		imshow("Processed", processed->image);
		imshow("Result", cv_ptr->image);
		
		waitKey(3);				
	}
	
	
	Mat processImage(Mat input)
	{
		Mat output;
		Mat HSV;
		cvtColor(input,HSV,COLOR_BGR2HSV);  //convert Image from RGB to HSV
		
		// Calibration of an optimal HSV------------------------------------------------------

		Mat CalibArea;
		vector<Mat> channels;
		double min, max;
		
		//Cut out CalibArea
		CalibArea= HSV(Rect(CalibAreaCornerX,CamYMax-CalibAreaHight,CalibAreaWidth,CalibAreaHight));
		imshow("Result2", CalibArea);
		split(CalibArea, channels);

		//Get H_min & H_max
		minMaxIdx(channels[0],&min,&max,NULL,NULL,Mat());
		H_max = max;
		if(max>42 && (max-min)>42)
		{
			H_min = max -42; // the Value 42 represents the whole spectrum of a color -> red->yellow->green->blue->magenta->red
		}
		else
		{
			H_min = min;
		}

		//Get S_min & S_max
		minMaxIdx(channels[1],&min,&max,NULL,NULL,Mat());
		S_max = max;
		S_min = min;
		
		//Get V_min & V_max
		minMaxIdx(channels[2],&min,&max,NULL,NULL,Mat());
		V_max = max;
		V_min = min;
		
		inRange(HSV,Scalar(H_min,S_min,V_min),Scalar(H_max,S_max,V_max),output);						
		//returns pixel in black&white. if the input pix are out of Range you use it returns black pix, in Range = white
				
		return output;
	}
				
	Mat processGaussian (Mat input)
	{
		Mat output;
						
		GaussianBlur(input,				// input image
					output,				// function output
					Size(15,15),			// smoothing window width and height in pixels
					0);					// sigma value, determines how much the image will be blurred
							
		return output;
	}

		
	Mat processHugh(Mat input,Mat color)
	{	
		float Position_x;
		float Position_y;
		float Radius;
		
		Mat output=color;
		vector<Vec3f> vecCircles;		// vector of type float		
		
		// Draw Area of Ball-detection and Calibration Area to Image
		rectangle(output,Point(minRadius,minRadius),Point((CamXMax-minRadius),(CamYMax-CalibAreaHight)),Scalar(0,255,0),1,8,0);  //Area of Balldetection
		rectangle(output,Point(CalibAreaCornerX,CamYMax-CalibAreaHight),Point(CalibAreaCornerX+CalibAreaWidth,CamYMax),Scalar(0,255,0),1,8,0);	//Calibration Area	
			
		// find all round object in the Binary Image
		HoughCircles(input,vecCircles,CV_HOUGH_GRADIENT,2,input.rows / 4,100,50,minRadius,maxRadius);	//Attention-> The Max r should not be too small	
					
		for(int i=0; i<vecCircles.size();i++)
		{

			Point center(cvRound(vecCircles[i][0]),cvRound(vecCircles[i][1]));
			
			Radius = cvRound(vecCircles[i][2]);			
			Position_x=cvRound(vecCircles[i][0]);
			Position_y=cvRound(vecCircles[i][1]);	
			Position_y=CamYMax - Position_y;
			
			//We accept just Balls, if they are in the Area of Balldetection
			if(Position_y < (CamYMax-minRadius) /* && Position_y > CalibAreaHight*/ && Position_x > minRadius && Position_x < (CamXMax-minRadius)) 
			{
				checkBall(output, Position_x, Position_y, Radius); // Überprüfe ob der neu eingelesene Ball schon bekannt ist
			}
		}
		
		deleteBall(); //delete all known Balls witch are not found in checkBall
		
		//Visualisation of known Balls in screen "Resault"
		if(known_balls.size()>0)
		{
			for(int i=0; i<known_balls.size(); i++)
			{
				Point center(known_balls[i].x, (CamYMax - known_balls[i].y));
				circle(output,center,3,Scalar(0,255,0),1,8,0);
				circle(output,center,known_balls[i].r,Scalar(0,0,255),1,8,0);							
				putText(output,"Ball "+Convert(known_balls[i].id)+"  "+ConvertD(ctX(known_balls[i].x))+"x"+"  "+ConvertD(ctY(known_balls[i].y))+"y", center, 1, 1, CV_RGB(125,12,145), 2);	
				//putText(output,"Ball "+Convert(known_balls[i].id)+"  x="+ConvertD(known_balls[i].x)+"  y="+ConvertD(known_balls[i].y), center, 1, 1, CV_RGB(125,12,145), 2);												
			}
		}
		return output;
	}	
		
	void checkBall(Mat input, int coordinate_x, int coordinate_y, int radius)
	{
		Mat output=input;
		
		float distance=0; // Zur Distanzberechnung zwischen neu eingelesenem und bekanntem Ball
		
		if(known_balls.size()>0)
		{			
			for(int i=0; i<known_balls.size(); i++) 
			{
				distance = sqrt(((known_balls[i].x - coordinate_x)*(known_balls[i].x - coordinate_x))+((known_balls[i].y - coordinate_y)*(known_balls[i].y - coordinate_y)));
				
				if(distance < radius)
				{
					known_balls[i].x = coordinate_x;
					known_balls[i].y = coordinate_y;

					putText(output,"Tracking ball",Point(20,20), 1, 1, CV_RGB(32,0,32), 2);
					
					known_balls[i].lastDetect = timestamp;
					return; // Beende die Funtkion an dieser Stelle, da der neu eingelesene Ball schon bekannt ist.
				}
			}
		}
		
		// New Ball 
		
		known_balls.push_back(Ball_Data());
		
		int n = known_balls.size()-1;
		known_balls[n].id = ballcounter;
		known_balls[n].x = coordinate_x;
		known_balls[n].y = coordinate_y;
		known_balls[n].r = radius;
		known_balls[n].firstDetect = timestamp;

		ballcounter++;

		printf("Neuer Ball erkannt!! %i \n\n",ballcounter);
											
		return;	 
	}
	
	
	void deleteBall()
	{
		int detectDuration = 0; //Duration since last Ball detection
		
		if(known_balls.size()>0)
		{
			for(int i=0; i<known_balls.size(); i++) 
			{
				detectDuration = timestamp - known_balls[i].lastDetect;
				
				if(detectDuration >= maxLostTime)
				{
					known_balls.erase(known_balls.begin()+i);
				}
			}
		}
		return;
	}
	

	double ctX(int xCoordinate) //Coordinate Transform for X-Value. Negativ for left and postiv for right position from the center of the camera
	{
		
		double distance;
		double partAngle;
		
		//Hier kann partAngle negativ werden tan(-Zahl)
		partAngle = (xCoordinate - CamXMax/2) * CamAoV/2 / CamYMax/2;	
		distance = CamHight * tan(partAngle * PI / 180.0 );
		
		return distance;	
	}
	

	double ctY(int yCoordinate) //Coordinate Transform
	{
		
		double distance;
		double partAngle;
		
		partAngle = (yCoordinate - CamYMax/2) * CamAoV/2 / CamYMax/2;
		//partAngle = yCoordinate * CamAoV / CamYMax;
		
		distance = CamHight * tan(partAngle * PI / 180.0 );
			
		return distance;
	}

	
	void publishKnownBalls()
	{
		//message for publishing the balls
		pcl::PointCloud<pcl::PointXYZ> pcl_ball_cloud;
		
	
		int knownTime = 0; //Time how long the Objekt is known
		
		if(known_balls.size()>0)
		{
			points.points.clear();
			
			for(int i=0; i<known_balls.size(); i++) 
			{
							
				knownTime = known_balls[i].lastDetect - known_balls[i].firstDetect;
				if(knownTime >= minKnownTime)
				{
					// set coordinates to point
					point.x = ctX(known_balls[i].x);
					point.y = ctY(known_balls[i].y);
					point.z = -CamHight;
					
					points.pose.position.x = point.x;
					points.pose.position.y = point.y;
					points.pose.position.z = point.z;
					// push point to pcl cloud
					pcl_ball_cloud.points.push_back(pcl::PointXYZ(point.x,point.y,point.z));
					points.header.stamp = ros::Time::now();
					points.points.push_back(point); 	
				}
			}
			//now convert pcl cloud to pointcloud2 and publish the values
			pcl_ball_cloud.width=pcl_ball_cloud.points.size();
			pcl_ball_cloud.height=1;
			sensor_msgs::PointCloud2 ball_cloud;	
			pcl::toROSMsg(pcl_ball_cloud,ball_cloud);
			ball_cloud.header.frame_id=points.header.frame_id;
			ball_cloud.header.stamp=ros::Time::now();
			cloud_pub.publish(ball_cloud);
			marker_pub.publish(points);
		}
		return;		
	}
		
	string ConvertD(double number)
	{
		std::ostringstream ss;
		ss << number;
		return ss.str();
	}	
	string Convert(float number)
	{
		std::ostringstream ss;
		ss << number;
		return ss.str();
	}
};


int main(int argc, char** argv) 
{
	ros::init(argc, argv, "ball detector node");
			
	ros::NodeHandle n("~");
	Ball_detect ball;	
	
	string image_raw,out1,out2, cloud_out;
	n.param<string>("image_raw",image_raw,"/raw");  
	n.param<string>("image_out1",out1,"/image1"); 
	n.param<string>("image_out2",out2,"/image2"); 
	n.param<int>("minRad",ball.minRadius,10); 
	n.param<int>("maxRad",ball.maxRadius,60); 
	n.param<double>("CamHight",ball.CamHight,0.4); 
	n.param<double>("CamAoV",ball.CamAoV,60.0);
	n.param<int>("CalibAreaCornerX",ball.CalibAreaCornerX,270);
	n.param<int>("CalibAreaHight",ball.CalibAreaHight,100);
	n.param<int>("CalibAreaWidth",ball.CalibAreaWidth,100);
	n.param<std::string>("balls_out", cloud_out, "/balls"); 
	
	ball.image1=n.advertise<sensor_msgs::Image>(out1.c_str(),100); //ist dies Teil eines Publisher??
	ball.image2=n.advertise<sensor_msgs::Image>(out2.c_str(),100);
	ball.marker_pub= n.advertise<visualization_msgs::Marker>("/DetectedBall_Marker", 10);
	ball.cloud_pub=n.advertise<sensor_msgs::PointCloud2>(cloud_out.c_str(), 1);
	
	ros::Subscriber imageSub;
	imageSub=n.subscribe(image_raw.c_str(),10,&Ball_detect::readImage,&ball);
	
	ros::spin();	
	return 0;	
	
}
