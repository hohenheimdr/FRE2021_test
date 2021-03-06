#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include <tf/transform_broadcaster.h>
#include "std_msgs/Float64.h"
#include <ros/subscriber.h>
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/TwistStamped.h"
#include "std_msgs/Bool.h"

#include <pcl_ros/point_cloud.h>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pointcloud_plantdetector/FilterConfig.h>
#include <dynamic_reconfigure/server.h>

#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/min_cut_segmentation.h>

/* 
 * Author: David Reiser 19.5.16
 * Description: read a 3D Pointcloud as a topic (can also be a 2D point cloud and filter out possible plant positions
 * with the help of a random forest algorithm
 * The plant positions get published as a filtered Point cloud with single positions of the estimated plant 
 * positions. These positions can afterwards be used to define plant spacing, row orientation and other 
 * usefull information for robot navigation and application
 * 
 * 
 * 24.2.17 added nan filter for point clouds
 * 
 *  * */
typedef pcl::PointCloud<pcl::PointXYZI> PCLCloud;


class DETECTOR
{
	private:
	pointcloud_plantdetector::FilterConfig conf;
	
	
	public:
	
	int neigbour_nr,minClusterSize;
	double radius, distance,ransac_dist;
	std::string frame_id;
	
	dynamic_reconfigure::Server<pointcloud_plantdetector::FilterConfig> server_;
	
// public variables
	ros::Publisher cloud_out_pub,filtercloud_out_pub;
	
	DETECTOR()
	{
		dynamic_reconfigure::Server<pointcloud_plantdetector::FilterConfig>::CallbackType f;
	    f = boost::bind(&DETECTOR::configure,this, _1, _2);
	    server_.setCallback(f);
	}
	
	~ DETECTOR()
	{
		
	}
	
	void configure(pointcloud_plantdetector::FilterConfig &config, uint32_t level) {
		
		ROS_INFO("reading dynamic reconfigure files");
		conf=config;
	}
	
	void readFrontCloud(const sensor_msgs::PointCloud2::ConstPtr& msg)
	{
		try{
		//create a pcl point cloud first...
		pcl::PointCloud<pcl::PointXYZI>::Ptr input (new pcl::PointCloud<pcl::PointXYZI>);
		//now there is some processing neccessary...convert msg to pcl point cloud		
		pcl::fromROSMsg(*msg, *input);
		//apply box filter
		input=box_filter(input);
		//apply ransac plane removal:
		input=remove_ground_plane(input);
		//now remove outliers
		input= apply_outlier_filter(input, neigbour_nr,radius);
		input=statistical_outlier_filter(input);
		//apply push to plane
		input=push_to_plane(input,1.0);
		
		//publish data after filtering:
		sensor_msgs::PointCloud2 cloud;
		//direcly write back the information to the point cloud msg
		pcl::toROSMsg(*input,cloud);
		//set the new header and size tranforms
		cloud.header.frame_id=frame_id;
		cloud.header.stamp=ros::Time::now();
		cloud.width=input->points.size();
		filtercloud_out_pub.publish(cloud);
		//cluster clouds with the given distance,and get out the center of every cluster
		pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud (new pcl::PointCloud<pcl::PointXYZ>);
		copyPointCloud(*input,*xyz_cloud);
		//variable use mincluster
		//xyz_cloud=min_cluster_clouds(xyz_cloud);
		xyz_cloud=nearest_cluster(xyz_cloud);
		//or eucledian cluster
		xyz_cloud=cluster_clouds(xyz_cloud);
		//filter nan points
		xyz_cloud=filter_nan_points(xyz_cloud);
		copyPointCloud(*xyz_cloud,*input);
		//publish the end data :)
		//direcly write back the information to the point cloud msg
		pcl::toROSMsg(*input,cloud);
		//set the new header and size tranforms
		cloud.header.frame_id=frame_id;
		cloud.header.stamp=ros::Time::now();
		cloud.width=input->points.size();
		cloud.height=1;
		cloud_out_pub.publish(cloud);
		
		}catch (...)
		{
			
		}
	}
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr filter_nan_points(pcl::PointCloud<pcl::PointXYZ>::Ptr input)
	{
		std::vector<int> indices;
		pcl::removeNaNFromPointCloud(*input,*input,indices);
		
		return input;	
	}
	
	pcl::PointCloud<pcl::PointXYZI>::Ptr box_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
	{
		try
		{
			pcl::PointCloud<pcl::PointXYZI>::Ptr output (new pcl::PointCloud<pcl::PointXYZI>);
			
			if(conf.use_box_filter)
			{
				for(int i=0; i<cloud->points.size();i++)
				{
					if(cloud->points[i].x<conf.x_max && cloud->points[i].x>conf.x_min)
					{
						if(cloud->points[i].y<conf.y_max && cloud->points[i].y>conf.y_min)
						{
							if(cloud->points[i].z<conf.z_max && cloud->points[i].z>conf.z_min)
							{
								output->points.push_back(cloud->points[i]);
							}
						}
					}	
				}
			}else
			{
				return cloud;
			}
			output->width=output->points.size();
			output->height=1;
			return output;
		}catch(...)
		{
			ROS_INFO("something went wrong...");
		}
	}
	
	pcl::PointCloud<pcl::PointXYZI>::Ptr new_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
	{
		//hier kommt der Code hin..
		
		return cloud;
	}
	
	pcl::PointCloud<pcl::PointXYZI>::Ptr statistical_outlier_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
	{
		if(conf.use_statistical_outlier_filter)
		{
			pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
			sor.setInputCloud (cloud);
			//number of neighbors to analyze
			sor.setMeanK (conf.meanK);
			//standard deviation multiplier
			sor.setStddevMulThresh (conf.stddev);
			sor.filter (*cloud);
		}
		return cloud;
	}

	pcl::PointCloud<pcl::PointXYZI>::Ptr push_to_plane(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, double z_value)
	{
		//define one z_value for all points in 3 and push them together
		if(conf.push_to_plane)
		{
			
			for(int i=0; i<cloud->points.size();i++)
			{
				cloud->points[i].z=z_value;
				//std::cout << "change z for :" <<cloud->points[i].x<<";"<<cloud->points[i].y <<" point " <<std::endl;
			}
		}
		return cloud;
	}
	//
	pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_clouds(pcl::PointCloud<pcl::PointXYZ>::Ptr input)
	{
		if(conf.use_eucledian_cluster)
		{
		 pcl::PointCloud<pcl::PointXYZ>::Ptr center_points (new pcl::PointCloud<pcl::PointXYZ>);
		 // Creating the KdTree object for the search method of the extraction
		  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
		  tree->setInputCloud (input);

		  std::vector<pcl::PointIndices> cluster_indices;
		  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
		  ec.setClusterTolerance (conf.cluster_distance); // in cm
		  ec.setMinClusterSize (conf.minClusterSize);
		  ec.setMaxClusterSize (conf.maxClusterSize);
		  ec.setSearchMethod (tree);
		  ec.setInputCloud (input);
		  ec.extract (cluster_indices);
	
		  int j = 0;
		  //now move trough all clusters of the point cloud...
		  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
		  {
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
			for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
			cloud_cluster->points.push_back (input->points[*pit]); 
			cloud_cluster->width = cloud_cluster->points.size ();
			cloud_cluster->height = 1;
			cloud_cluster->is_dense = true;

			std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
			//compute centroid of point cloud
			pcl::PointXYZ center;
			Eigen::Vector4f centroid; 
			pcl::compute3DCentroid(*cloud_cluster,centroid); 
			center.x=centroid[0];
			center.y=centroid[1];
			center.z=centroid[2];
			center_points->push_back(center);
			
			j++;
		  }
		
		center_points->width=center_points->points.size();
		
		//return input;
		return center_points;
		}else
		{
			return input;
		}
		
	}
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr nearest_cluster(pcl::PointCloud<pcl::PointXYZ>::Ptr input)
	{
		//if(conf.use_eucledian_cluster)
		//{
		 pcl::PointCloud<pcl::PointXYZ>::Ptr center_points (new pcl::PointCloud<pcl::PointXYZ>);
		 // Creating the KdTree object for the search method of the extraction
		  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
		  tree->setInputCloud (input);

		  std::vector<pcl::PointIndices> cluster_indices;
		  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
		  ec.setClusterTolerance (conf.cluster_distance); // in cm
		  ec.setMinClusterSize (conf.minClusterSize);
		  ec.setMaxClusterSize (conf.maxClusterSize);
		  ec.setSearchMethod (tree);
		  ec.setInputCloud (input);
		  ec.extract (cluster_indices);
	
		  int j = 0;
		  double actual_dist=10.0;
		  //now move trough all clusters of the point cloud...
		  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
		  {
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
			for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
			cloud_cluster->points.push_back (input->points[*pit]); 
			cloud_cluster->width = cloud_cluster->points.size ();
			cloud_cluster->height = 1;
			cloud_cluster->is_dense = true;

			std::cout <<"no.: "<<j<< "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
			//compute centroid of point cloud
			pcl::PointXYZ center;
			Eigen::Vector4f centroid; 
			pcl::compute3DCentroid(*cloud_cluster,centroid); 
			center.x=centroid[0];
			center.y=centroid[1];
			center.z=centroid[2];
			//center_points->push_back(center);
			j++;
			double dist=sqrt((center.x-conf.x)*(center.x-conf.x)+(center.y-conf.y)*(center.y-conf.y));
			if(dist<actual_dist)
			{
				center_points=cloud_cluster;
				actual_dist=dist;
			}
						
		  }
		center_points->width=center_points->points.size();
		center_points->height=1;
		
		if (center_points->points.size()>0)
		{
			return center_points;	
		}else
		{
			return input;
		}
		
	}
	
		
	pcl::PointCloud<pcl::PointXYZI>::Ptr remove_ground_plane(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
	{
		if(conf.remove_plane)
		{
			//see also 	pcl  cluster extraction tutorial
			 pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZI>);
			// Create the segmentation object for the planar model and set all the parameters
			  pcl::SACSegmentation<pcl::PointXYZI> seg;
			  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
			  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
			  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZI> ());
			  pcl::PCDWriter writer;
			  seg.setOptimizeCoefficients (true);
			  seg.setModelType (pcl::SACMODEL_PLANE);
			  seg.setMethodType (pcl::SAC_RANSAC);
			  seg.setMaxIterations (conf.max_iterations);
			  seg.setDistanceThreshold (conf.ransac_dist);

			  int i=0, nr_points = (int) cloud->points.size ();
			  while (cloud->points.size () > 0.3 * nr_points)
			  {
				// Segment the largest planar component from the remaining cloud
				seg.setInputCloud (cloud);
				seg.segment (*inliers, *coefficients);
				if (inliers->indices.size () == 0)
				{
				  std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
				  break;
				}

				// Extract the planar inliers from the input cloud
				pcl::ExtractIndices<pcl::PointXYZI> extract;
				extract.setInputCloud (cloud);
				extract.setIndices (inliers);
				extract.setNegative (false);

				// Get the points associated with the planar surface
				extract.filter (*cloud_plane);
				std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

				// Remove the planar inliers, extract the rest
				extract.setNegative (true);
				extract.filter (*cloud_f);
				*cloud = *cloud_f;
				}
		}

		return cloud;
	
	}
	//-----------------------------------------------------------------------
	pcl::PointCloud<pcl::PointXYZI>::Ptr apply_outlier_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,int nr, double radius)
	{
		if(conf.use_radius_outlier_filter)
		{
			pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);
			
			pcl::RadiusOutlierRemoval<pcl::PointXYZI> outrem;
			// build the filter
			outrem.setInputCloud(cloud);
			outrem.setRadiusSearch(conf.radius);
			outrem.setMinNeighborsInRadius (conf.neigbour_nr);
			// apply filter
			outrem.filter (*cloud_filtered);
	
			std::cerr << "Cloud before filtering: " << cloud->points.size()<< "and after " <<cloud->points.size()<< std::endl;
			return cloud_filtered;
		}else
		{
			return cloud;
		}		
		
		
		
	}
	
};
//-------------------------------------------------------------------------

int main(int argc, char** argv){

ros::init(argc, argv, "error_laser_cmd");

ros::NodeHandle n("~");

 DETECTOR o;
 
	  std::string front_str,cloud_out,filtercloud_out;
	ROS_INFO("starting pointcloud_ filter");
	  n.param<std::string>("cloud_in", front_str, "/error_cloud_front"); 
	  n.param<std::string>("frame_id",o.frame_id,"odom");
	  n.param<std::string>("plants_pub", cloud_out, "plants_out"); 
	  n.param<std::string>("filtered_cloud", filtercloud_out, "/no_plane");
	  //n.param<double>("ransac_dist", o.ransac_dist, 0.05);
	  //n.param<double>("radius", o.radius, 5);
	  //n.param<int>("neigbour_nr", o.neigbour_nr, 5);
	  //n.param<double>("distance", o.distance, 10);
	  //n.param<int>("minClusterSize", o.minClusterSize, 10);
	  	  
	  ros::Subscriber sub_laser_front = n.subscribe(front_str,1, &DETECTOR::readFrontCloud, &o);
      //create publishers
      o.cloud_out_pub=n.advertise<sensor_msgs::PointCloud2>(cloud_out.c_str(), 1);
      o.filtercloud_out_pub=n.advertise<sensor_msgs::PointCloud2>(filtercloud_out.c_str(), 1);
 
	ros::spin();
  

}
