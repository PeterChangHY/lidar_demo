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

ros::Publisher pub_bg;
ros::Publisher pub_non_bg;

pcl::PointCloud<pcl::PointXYZ>::Ptr background(new pcl::PointCloud<pcl::PointXYZ>);

float resolution = 1280.0f;
float radius = 0.3f;
pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution);
//pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree( resolution);
bool flag = false;
bool update_flag = false;
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
  sor.setLeafSize (0.1f, 0.1f, 0.1f);
  sor.filter (*cloud_filtered);
	
  //update background
  if(update_flag){
	*background += *cloud_filtered;
        //octree.setInputCloud (background);
  	//octree.addPointsFromInputCloud();
        update_count++;
        if(update_count >= 3 ) {
           update_flag = false;
           update_count = 0;
            octree.setInputCloud (background);
           octree.addPointsFromInputCloud();
           }
        //octree.switchBuffers();
  }
  
  //octree.setInputCloud(cloud_filtered);
  //octree.addPointsFromInputCloud();
  
  //std::vector<int> newPointIdxVector;
  //pcl::PointIndices::Ptr newPointIdices(new pcl::PointIndices);
  //octree.getPointIndicesFromNewVoxels(newPointIdxVector);
  /*for(auto idx:newPointIdxVector){
	newPointIdices->push_back(idx);	
	}
   
  pcl::PointCloud<pcl::PointXYZ>::Ptr bg_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr non_bg_cloud(new pcl::PointCloud<pcl::PointXYZ>);  

  pcl::ExtractIndices<pcl::PointXYZ > eifilter_bg(true);
  eifilter_bg.setInputCloud(cloud_filtered);
  eifilter_bg.setIndices(newPointIdices);
  eifilter_bg.filter(*bg_cloud);

   pcl::ExtractIndices<pcl::PointXYZ > eifilter_non_bg(false);
  eifilter_non_bg.setInputCloud(cloud_filtered);
  eifilter_non_bg.setIndices(newPointIdxVector);
  eifilter_non_bg.filter(*non_bg_cloud);
  */
  pcl::PointCloud<pcl::PointXYZ>::Ptr bg_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr non_bg_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  //filtered the backgound
  if(flag && !update_flag){  
      std::vector<int> pointIdxRadiusSearch;
      std::vector<float> pointRadiusSquaredDistance;
      for(auto& p:cloud_filtered->points){
          if (octree.radiusSearch (p, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
  	    {
		//bg_cloud->push_back(p);
	    }
           else{
		non_bg_cloud->push_back(p);
		}
      }
  bg_cloud = background;
  /*for(auto idx:newPointIdxVector){
	non_bg_cloud->push_back(cloud_filtered->points[idx]);
  }
  bg_cloud = cloud_filtered;	
  */ 
  //Convert to ROS data type
  sensor_msgs::PointCloud2 bg_msg;
  sensor_msgs::PointCloud2 non_bg_msg;
  pcl::toROSMsg(*bg_cloud,bg_msg);
  pcl::toROSMsg(*non_bg_cloud,non_bg_msg);
  bg_msg.header.frame_id = input->header.frame_id;
  non_bg_msg.header.frame_id = input->header.frame_id;
  pub_bg.publish(bg_msg);
  pub_non_bg.publish(non_bg_msg);
  }

}

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s], so update the background", msg->data.c_str());
  update_flag = true;
  flag = true;
}


int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub_bg = nh.advertise<sensor_msgs::PointCloud2> ("background", 1);
  pub_non_bg = nh.advertise<sensor_msgs::PointCloud2> ("non_background", 1);

  ros::Subscriber sub_back = nh.subscribe("update_bg", 1000, chatterCallback);
  ros::spin();

}
