#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree.h>
#include "std_msgs/String.h"
#include <vector>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <omp.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <cstdlib>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/features/moment_of_inertia_estimation.h>

ros::Publisher pub_bg;
ros::Publisher pub_non_bg;
ros::Publisher pub_non_bg_bbox;

pcl::PointCloud<pcl::PointXYZ>::Ptr background(new pcl::PointCloud<pcl::PointXYZ>);


float resolution = 1280.0f;
float radius = 0.3f;
//pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution);
pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
//pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree( resolution);
bool flag = true;
bool update_flag = true;
int update_count = 0;
void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{	
  // Container for original & filtered data
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZ> );
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered( new pcl::PointCloud<pcl::PointXYZ> );
  
  //pcl::PCLPointCloud2* cloud_tmp = new pcl::PCLPointCloud2; 
  //pcl::PCLPointCloud2ConstPtr cloudPtr(cloud_tmp);
  // Convert to PCL data type
  //pcl_conversions::toPCL(*input, *cloudPtr);
  //pcl::fromPCLPointCloud2(*cloudPtr, *cloud ); 
  
  pcl::fromROSMsg(*input, *cloud);	
  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PointXYZ > sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (0.2f, 0.2f, 0.2f); //20 cm
  sor.filter (*cloud_filtered);
	
  //update background
  if(update_flag){
	*background += *cloud_filtered;
        //octree.setInputCloud (background);
  	//octree.addPointsFromInputCloud();
        update_count++;
        if(update_count >= 4 ) {
           update_flag = false;
           update_count = 0;
           //octree.setInputCloud (background);
           //octree.addPointsFromInputCloud();
            kdtree.setInputCloud (background);
	}
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr bg_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr non_bg_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr non_bg_cloud_color(new pcl::PointCloud<pcl::PointXYZRGB>);
  //filtered the backgound
  
  if(!update_flag){  
      std::vector<int> pointIdxRadiusSearch;
      std::vector<float> pointRadiusSquaredDistance;
      #pragma omp for
      for(auto& p:cloud_filtered->points){
          if (kdtree.radiusSearch (p, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
  	    {
		//bg_cloud->push_back(p);
	    }
           else{
		non_bg_cloud->push_back(p);
		}
  	}
  // cluster
  //clean old markerArray
  visualization_msgs::MarkerArray markerArray_clean;
  visualization_msgs::Marker marker;
  marker.header.frame_id = input->header.frame_id;
  marker.action = visualization_msgs::Marker::DELETEALL;
  markerArray_clean.markers.push_back(marker);
  pub_non_bg_bbox.publish(markerArray_clean);
  

  if(non_bg_cloud->size() > 0){ 
         std::stringstream ss1;
         ss1 << non_bg_cloud->size();
         ROS_INFO("non bg point size: %s",ss1.str().c_str());
	  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	  tree->setInputCloud (non_bg_cloud);

	  std::vector<pcl::PointIndices> cluster_indices;
	  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	  ec.setClusterTolerance (0.3); // 30cm
	  ec.setMinClusterSize (2);
	  ec.setMaxClusterSize (200);
	  ec.setSearchMethod (tree);
	  ec.setInputCloud (non_bg_cloud);
	  ec.extract (cluster_indices);
          visualization_msgs::MarkerArray markerarray;
	  int id = 0;
	  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	  {
	    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
	    int r = rand() % 256;
	    int g = rand() % 256;
	    int b = rand() % 256;	
	    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
		 pcl::PointXYZRGB color_point(r,g,b);
		 color_point.x = non_bg_cloud->points[*pit].x;
		 color_point.y = non_bg_cloud->points[*pit].y;
		 color_point.z = non_bg_cloud->points[*pit].z;
		 cloud_cluster->push_back(color_point);
	  	}
	    cloud_cluster->width = cloud_cluster->size ();
            
	    if (cloud_cluster->size() >= 8){
		*non_bg_cloud_color += *cloud_cluster;
		//bbox
                pcl::MomentOfInertiaEstimation <pcl::PointXYZRGB> feature_extractor;
  		feature_extractor.setInputCloud (cloud_cluster);
                feature_extractor.compute ();     
 		pcl::PointXYZRGB min_point_AABB;
                pcl::PointXYZRGB max_point_AABB;
                feature_extractor.getAABB (min_point_AABB, max_point_AABB);
                visualization_msgs::Marker marker;
                marker.header.stamp = ros::Time();
                marker.ns = "bbox";
                marker.id = id; id++;
                marker.frame_locked = false;
                marker.type = visualization_msgs::Marker::CUBE;
                marker.action = visualization_msgs::Marker::ADD;
                marker.header.frame_id = input->header.frame_id;
                marker.pose.position.x = (min_point_AABB.x + max_point_AABB.x)/2;
                marker.pose.position.y = (min_point_AABB.y + max_point_AABB.y)/2;
                marker.pose.position.z = (min_point_AABB.z + max_point_AABB.z)/2;
 		marker.scale.x = 0.7;
		marker.scale.y = 0.7;
		marker.scale.z = 1.7;
		marker.color.a = 0.2;
                marker.color.r = 0.8;
                marker.color.g = 0.4;
                marker.color.b = 0.4;
                marker.pose.orientation.w = 1.0;
                markerarray.markers.push_back(marker);
	    }
	  }
          pub_non_bg_bbox.publish(markerarray);

         std::stringstream ss,ss2;
         ss << non_bg_cloud_color->size();
         ROS_INFO("color point size: %s",ss.str().c_str());
         ss2 << markerarray.markers.size();
         ROS_INFO("markerarray size: %s",ss2.str().c_str());
  }
  bg_cloud = background;


  //Convert to ROS data type
  sensor_msgs::PointCloud2 bg_msg;
  sensor_msgs::PointCloud2 non_bg_msg;
  pcl::toROSMsg(*bg_cloud,bg_msg);
  pcl::toROSMsg(*non_bg_cloud_color,non_bg_msg);
  bg_msg.header.frame_id = input->header.frame_id;
  non_bg_msg.header.frame_id = input->header.frame_id;
  pub_bg.publish(bg_msg);
  pub_non_bg.publish(non_bg_msg);
  }

}


int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;
  std::string input_topic;
  if ( !nh.getParam("input_topic", input_topic)){
	input_topic = "/velodyne_points";
  }

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe (input_topic, 1, cloud_cb);
  ROS_INFO("input: %s",input_topic.c_str());
  // Create a ROS publisher for the output point cloud
  pub_bg = nh.advertise<sensor_msgs::PointCloud2> ("background", 1);
  pub_non_bg = nh.advertise<sensor_msgs::PointCloud2> ("non_background", 1);
  //bbox
  pub_non_bg_bbox = nh.advertise<visualization_msgs::MarkerArray> ("non_background_bbox", 10);

  ros::spin();

}
