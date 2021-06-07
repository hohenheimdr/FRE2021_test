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
#include <std_msgs/Bool.h>


// comments: 
//dr 	08.06.17	added output as pointcloud

//todos: make picture representation optional, best with launchfile option, LaunchFile name marker publisher
//include output of std::bool when ball was detected
//include option to push the reference frame to a x+ level#
//output result picture as message

//ToDo: Launch File Publischer usw aufnehmen!!





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
	bool published;
	
}; 

class Ball_detect
{
	private:
	
	cv_bridge::CvImagePtr cv_ptr,processed;	
		
	public:
	
	int H_min,H_max, S_min,S_max,V_min,V_max;
	int minKnownTime, maxLostTime, minRadius, maxRadius;
	int CalibAreaX, CalibAreaY ,CalibAreaHight, CalibAreaWidth, DetectionAreaHight;
	int AreaL_X, AreaL_W, AreaM_X, AreaM_W, AreaR_X, AreaR_W, Areas_Y, Areas_Hight; 
	int ballcounter;
	
	time_t timestamp;
	
	
	double CamHight;
	double CamAoV; 		//Cam Angle of View
	int CamXMax;		// Maße des USB Cam Images mit dem Format 4:3
	int CamYMax;
	
	int HoughGradient, HoughCanny, HoughCenter;
	int GaussianSigmaX, GaussianSigmaY, GaussianSize;
	
	vector<Ball_Data> known_balls;
	
	ros::Publisher image1,image2;
	ros::Publisher cloud_pub, balldetect_pub, ballInAreaL_pub, ballInAreaM_pub, ballInAreaR_pub;
	geometry_msgs::Point point;
	
	string frameID;

	Ball_detect() //constructor in Form einer Funktion
	{
		// Bereich für Parameter Einstellungen!!

		H_min=0;
		S_min=0;
		V_min=0;
		H_max=0;
		S_max=0;
		V_max=0;
		
		minRadius = 20;
		maxRadius = 30;
		
		HoughGradient = 2;
		HoughCanny = 150;
		HoughCenter = 20;
		GaussianSigmaX = 10;
		GaussianSigmaY = 10;
		GaussianSize = 15;

		timestamp = time(0);
		minKnownTime = 0; //Time how long the Objekt should be known before publishing
		maxLostTime = 3; //Max Seconds a Ball was not detect;
		
		CalibAreaX=425;
		CalibAreaY=400;
		CalibAreaHight=26;
		CalibAreaWidth=25;	
		DetectionAreaHight=400;
		
		Areas_Hight = 200;
		Areas_Y = CalibAreaY-Areas_Hight;
		AreaL_X = 0;
		AreaL_W = 220;
		AreaM_X = AreaL_X+AreaL_W;
		AreaM_W = 200;
		AreaR_X = AreaM_X+AreaM_W; 
		AreaR_W = 220;
		
		CamHight = 0.65;
		CamAoV = 67.58331136;
		CamXMax=640;
		CamYMax=480;
		
		ballcounter=0;

		namedWindow("Processed");
		namedWindow("Color_scale");
		namedWindow("Result");
		namedWindow("Result2");
 		/// Create Trackbars for Manual HSV Filter Setup
 		createTrackbar("minRadius","Color_scale",&minRadius,100,NULL);
 		createTrackbar("maxRadius","Color_scale",&maxRadius,200,NULL);
 		createTrackbar("CalibAreaX","Color_scale",&CalibAreaX,CamXMax,NULL);
 		createTrackbar("CalibAreaY","Color_scale",&CalibAreaY,CamYMax,NULL);
 		createTrackbar("CalibAreaHight","Color_scale",&CalibAreaHight,CamYMax,NULL);
 		createTrackbar("CalibAreaWidth","Color_scale",&CalibAreaWidth,CamXMax,NULL);
 		createTrackbar("HoughGradient","Color_scale",&HoughGradient,10,NULL);
 		createTrackbar("HoughCanny","Color_scale",&HoughCanny,300,NULL);
 		createTrackbar("HoughCenter","Color_scale",&HoughCenter,300,NULL);
 		createTrackbar("GaussianSigmaX","Color_scale",&GaussianSigmaX,10,NULL);
 		createTrackbar("GaussianSigmaY","Color_scale",&GaussianSigmaY,10,NULL);
 		createTrackbar("GaussianSize","Color_scale",&GaussianSize,99,NULL);
 		createTrackbar("DetectionAreaHight","Color_scale",&DetectionAreaHight,CamYMax,NULL);

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
		
		frameID = msg->header.frame_id;
		
		checkSizeOfAreas();
		
		processed->image=processImage(cv_ptr->image);			

		processed->image=processGaussian (processed->image);
		image2.publish(processed->toImageMsg());
		
		cv_ptr->image=processHugh(processed->image,cv_ptr->image);
		image1.publish(cv_ptr->toImageMsg());	

		publishKnownBalls();
		deleteBall(); //delete all known Balls witch are not found in checkBall

		imshow("Processed", processed->image);
		imshow("Result", cv_ptr->image);
		
		waitKey(3);				
	}
	
	void checkSizeOfAreas()
	{
		if(CalibAreaX >= CamXMax)
		{
			CalibAreaX = CamXMax -1;
		}
		if(CalibAreaX+CalibAreaWidth > CamXMax)
		{
			CalibAreaWidth = CamXMax-CalibAreaX;	
		}
		
		if(CalibAreaY >= CamYMax)
		{
			CalibAreaY = CamYMax -1;
		}
		if(CalibAreaY+CalibAreaHight > CamYMax)
		{
			CalibAreaHight = CamYMax-CalibAreaY;
		}
	
		return;
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
		CalibArea= HSV(Rect(CalibAreaX,CalibAreaY,CalibAreaWidth,CalibAreaHight));
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
		
		printf("CalibValues:\n ");
		printf("H_max: %i\n " ,H_max);
		printf("H_min: %i\n " ,H_min);
		printf("S_max: %i\n " ,S_max);
		printf("S_min: %i\n " ,S_min);
		printf("V_max: %i\n " ,V_max);
		printf("V_min: %i\n " ,V_min);
				
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
		
		// Draw Area of Ball-detection, Calibration Area and Spraying-Areas to Image
		rectangle(output,Point(minRadius,minRadius),Point(CamXMax-minRadius,DetectionAreaHight),Scalar(0,255,0),1,8,0);  //Area of Balldetection
		rectangle(output,Point(CalibAreaX,CalibAreaY),Point(CalibAreaX+CalibAreaWidth,CalibAreaY+CalibAreaHight),Scalar(0,255,0),1,8,0);	//Calibration Area	
		rectangle(output,Point(AreaL_X,Areas_Y),Point(AreaL_X+AreaL_W,Areas_Y+Areas_Hight),Scalar(0,255,0),1,8,0);	//Spraying-Area Left
		rectangle(output,Point(AreaM_X,Areas_Y),Point(AreaM_X+AreaM_W,Areas_Y+Areas_Hight),Scalar(0,255,0),1,8,0);	//Spraying-Area Left
		rectangle(output,Point(AreaR_X,Areas_Y),Point(AreaR_X+AreaR_W,Areas_Y+Areas_Hight),Scalar(0,255,0),1,8,0);	//Spraying-Area Left
		
		// find all round object in the Binary Image
		HoughCircles(input,vecCircles,CV_HOUGH_GRADIENT,HoughGradient,input.rows / 4,HoughCanny,HoughCenter,minRadius,maxRadius);	//Attention-> The Max r should not be too small	
		
		int size = vecCircles.size();
		printf("no Circles: %i\n " ,size);
		/*
		 * HoughCircles( src_gray, circles, CV_HOUGH_GRADIENT, 1, src_gray.rows/8, 200, 100, 0, 0 );
		 * src_gray: Input image (grayscale)
		 * circles: A vector that stores sets of 3 values: x_{c}, y_{c}, r for each detected circle.
		 * CV_HOUGH_GRADIENT: Define the detection method. Currently this is the only one available in OpenCV
		 * dp = 1: The inverse ratio of resolution
		 * min_dist = src_gray.rows/8: Minimum distance between detected centers
		 * param_1 = 200: Upper threshold for the internal Canny edge detector
		 * param_2 = 100*: Threshold for center detection.
		 * min_radius = 0: Minimum radio to be detected. If unknown, put zero as default.
		 * max_radius = 0: Maximum radius to be detected. If unknown, put zero as default
		 */				
	
		for(int i=0; i<vecCircles.size();i++)
		{

			Point center(cvRound(vecCircles[i][0]),cvRound(vecCircles[i][1]));
			
			Radius = cvRound(vecCircles[i][2]);			
			Position_x=cvRound(vecCircles[i][0]);
			Position_y=cvRound(vecCircles[i][1]);	
			Position_y=CamYMax - Position_y; // correct (rotate) y-Coordinates to Bottom-Value: 0 Top-Value: Max (480 Pixel) of the Image
			
			//We accept just Balls, if they are in the Area of Balldetection
			if(Position_y < (CamYMax-minRadius) && Position_y > (CamYMax-DetectionAreaHight) && Position_x > minRadius && Position_x < (CamXMax-minRadius)) 
			{
				checkBall(output, Position_x, Position_y, Radius); // Überprüfe ob der neu eingelesene Ball schon bekannt ist
			}
		}
		
		//Visualisation of known Balls in screen "Resault"
		if(known_balls.size()>0)
		{
			for(int i=0; i<known_balls.size(); i++)
			{
				Point center(known_balls[i].x, (CamYMax - known_balls[i].y));
				circle(output,center,3,Scalar(0,255,0),1,8,0);
				circle(output,center,known_balls[i].r,Scalar(0,0,255),1,8,0);							
				putText(output,"Ball "+Convert(known_balls[i].id)+"  "+ConvertD(ctX(known_balls[i].x))+"x"+"  "+ConvertD(ctY(known_balls[i].y))+"y", center, 1, 1, CV_RGB(125,12,145), 2);	
			}
		}
		
		return output;
	}	
		
	void checkBall(Mat input, int coordinate_x, int coordinate_y, int radius)
	{
		Mat output=input;
		
		//float distance=0; // Zur Distanzberechnung zwischen neu eingelesenem und bekanntem Ball
		
		/*if(known_balls.size()>0)
		//{			
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
		}*/
		
		// New Ball 
		
		known_balls.push_back(Ball_Data());
		
		int n = known_balls.size()-1;
		known_balls[n].id = ballcounter;
		known_balls[n].x = coordinate_x;
		known_balls[n].y = coordinate_y;
		known_balls[n].r = radius;
		known_balls[n].firstDetect = timestamp;
		known_balls[n].published = false;

		ballcounter++;

		//printf("Neuer Ball erkannt!! %i \n\n",ballcounter);
											
		return;	 
	}
	
	
	void deleteBall()
	{
		int detectDuration = 0; //Duration since last Ball detection
		
		if(known_balls.size()>0)
		{
			for(int i=0; i<known_balls.size(); i++) 
			{
				//detectDuration = timestamp - known_balls[i].lastDetect;
				
				//if(detectDuration >= maxLostTime)
				//{
					known_balls.erase(known_balls.begin()+i);
				//}
			}
		}
		return;
	}
	

	double ctX(int xCoordinate) //Coordinate Transform for X-Value. Negativ for left and postiv for right position from the center of the camera
	{
		
		double distance;
		double partAngle;
		
		partAngle = (xCoordinate - CamXMax/2) * CamAoV/2 / CamYMax/2;	
		distance = CamHight * tan(partAngle * PI / 180.0 );
		
		return distance;	
	}
	

	double ctY(int yCoordinate) //Coordinate Transform for Y-Value. Negativ for bottom and positiv for top position from the center of the camera
	{
		
		double distance;
		double partAngle;
		
		partAngle = (yCoordinate - CamYMax/2) * CamAoV/2 / CamYMax/2;		
		distance = CamHight * tan(partAngle * PI / 180.0 );
			
		return distance;
	}


	void publishKnownBalls()
	{
		//message for publishing the balls
		pcl::PointCloud<pcl::PointXYZ> pcl_ball_cloud;
	
		int knownTime = 0; //Time how long the Objekt is known
		
		std_msgs::Bool out;

		if(known_balls.size()>0)
		{		
			out.data=true;
			int i =0;
			//for(int i=0; i<known_balls.size(); i++) 
			//{
				ROS_INFO("ball in Picture");		
				//knownTime = known_balls[i].lastDetect - known_balls[i].firstDetect;
				//if(knownTime >= minKnownTime)
				//{
					if(known_balls[i].published == false)
					{
						balldetect_pub.publish(out);
						known_balls[i].published = true;
					}
					if(/*known_balls[i].y < (CamYMax-Areas_Y) && */known_balls[i].y > (CamYMax-(Areas_Y+Areas_Hight)))
					{
						ROS_INFO("ball in area");
						if(known_balls[i].x > AreaL_X && known_balls[i].x < AreaM_X)
						{
							ROS_INFO("left_area ball");
							ballInAreaL_pub.publish(out);
						}
						if(known_balls[i].x > AreaM_X && known_balls[i].x < AreaR_X)
						{
							ROS_INFO("middle_area ball");
							ballInAreaM_pub.publish(out);
						}
						if(known_balls[i].x > AreaR_X && known_balls[i].x < CamXMax)
						{
							ROS_INFO("right_area ball");
							ballInAreaR_pub.publish(out);
						}
					}/*else
					{
						//out.data=false;
						//ballInAreaL_pub.publish(out);
						//ballInAreaM_pub.publish(out);
						//ballInAreaR_pub.publish(out);
					}*/
				//}
					// set coordinates to point
					point.x = ctX(known_balls[i].x);
					point.y = ctY(known_balls[i].y);
					point.z = -CamHight;
					
					// push point to pcl cloud
					pcl_ball_cloud.points.push_back(pcl::PointXYZ(point.x,point.y,point.z));	
			//}
			
			//now convert pcl cloud to pointcloud2 and publish the values
			pcl_ball_cloud.width=pcl_ball_cloud.points.size();
			pcl_ball_cloud.height=1;
			sensor_msgs::PointCloud2 ball_cloud;	
			pcl::toROSMsg(pcl_ball_cloud,ball_cloud);
			ball_cloud.header.frame_id=frameID;
			ball_cloud.header.stamp=ros::Time::now();
			cloud_pub.publish(ball_cloud);
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
	
	string image_raw,out1,out2, cloud_out, balldetect_out, ballInAreaL_out, ballInAreaM_out, ballInAreaR_out;
	n.param<string>("image_raw",image_raw,"/raw");  
	n.param<string>("image_out1",out1,"/image1"); 
	n.param<string>("image_out2",out2,"/image2"); 
	n.param<int>("minRad",ball.minRadius,10); 
	n.param<int>("maxRad",ball.maxRadius,60); 
	n.param<double>("CamHight",ball.CamHight,0.4); 
	n.param<double>("CamAoV",ball.CamAoV,60.0);
	n.param<int>("CalibAreaX",ball.CalibAreaX,270);
	n.param<int>("CalibAreaY",ball.CalibAreaY,380);
	n.param<int>("CalibAreaHight",ball.CalibAreaHight,100);
	n.param<int>("CalibAreaWidth",ball.CalibAreaWidth,100);
	n.param<int>("DetectionAreaHight",ball.DetectionAreaHight,400);
	n.param<int>("HoughGradient",ball.HoughGradient,2);
	n.param<int>("HoughCanny",ball.HoughCanny,150);
	n.param<int>("HoughCenter",ball.HoughCanny,20);
	n.param<int>("GaussianSigmaX",ball.GaussianSigmaX,10);
	n.param<int>("GaussianSigmaY",ball.GaussianSigmaY,10);
	n.param<int>("GaussianSize",ball.GaussianSize,15);
	n.param<int>("minKnownTime",ball.minKnownTime,0);
	n.param<int>("Areas_Hight",ball.Areas_Hight,200);
	
	n.param<std::string>("balls_out", cloud_out, "/balls"); 
	n.param<std::string>("balldetect_out", balldetect_out, "/Golfball"); 
	n.param<std::string>("ballInAreaL_out", ballInAreaL_out, "/left_sprayer"); 
	n.param<std::string>("ballInAreaM_out", ballInAreaM_out, "/middle_sprayer"); 
	n.param<std::string>("ballInAreaR_out", ballInAreaR_out, "/right_sprayer"); 

	
	ball.image1=n.advertise<sensor_msgs::Image>(out1.c_str(),100);
	ball.image2=n.advertise<sensor_msgs::Image>(out2.c_str(),100);
	ball.cloud_pub=n.advertise<sensor_msgs::PointCloud2>(cloud_out.c_str(), 1);
	ball.balldetect_pub=n.advertise<std_msgs::Bool>(balldetect_out.c_str(), 10);
	ball.ballInAreaL_pub=n.advertise<std_msgs::Bool>(ballInAreaL_out.c_str(), 10);
	ball.ballInAreaM_pub=n.advertise<std_msgs::Bool>(ballInAreaM_out.c_str(), 10);
	ball.ballInAreaR_pub=n.advertise<std_msgs::Bool>(ballInAreaR_out.c_str(), 10);
	
	ros::Subscriber imageSub;
	imageSub=n.subscribe(image_raw.c_str(),10,&Ball_detect::readImage,&ball);
	
	ros::spin();	
	return 0;	
	
}
