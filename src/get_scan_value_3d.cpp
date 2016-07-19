// ROS specific includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>               // For passthrough filter
#include <pcl/PCLPointCloud2.h>          // For passthrough filter
#include <pcl/filters/passthrough.h>     // For passthrou_filter
#include <pcl/point_types.h>             // For passthrou_filter
#include <pcl/point_cloud.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/impl/conditional_euclidean_clustering.hpp>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>

#include <sys/time.h>

using namespace std;


ros::Publisher pub; // For PointCloud output 
tf::TransformListener *tf_listener; // Transform pointcloud from LIDAR fixed link to base_link


void cloud_cb (const sensor_msgs::PointCloud2Ptr& entered_cloud)
{

/*****     Create a container for the data     *****/
	sensor_msgs::PointCloud2 transformed_cloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>); // 
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
	//sensor_msgs::PointCloud2 output_cloud; // For PointCloud output 


/*****     Transform PointCloud from LIDAR fixed link to base_link     *****/
	std::string target_link_name = "/base_link";
	tf::StampedTransform transform;
	try{
		tf_listener->waitForTransform(target_link_name, entered_cloud->header.frame_id, ros::Time(), ros::Duration(100.0));
	}catch(tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
	}
	
	if(!pcl_ros::transformPointCloud(target_link_name, *entered_cloud, transformed_cloud, *tf_listener)){
		return;
	}
	

/*****     Conversion PointCloud2 intensity field name to PointXYZI intensity field name     *****/
	transformed_cloud.fields[3].name = "intensity";
	pcl::fromROSMsg(transformed_cloud, *cloud);
  
    std::cerr << "Cloud before filtering: " << std::endl;
    std::cerr << *cloud << std::endl;
/*****     PassThrough process      *****/
    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 1.0); // Range of filter value is between 0m and 1m
    pass.setFilterLimitsNegative (false);
    pass.filter (*cloud_filtered);
    //pass.filter (*cloud); // same name input cloud is error 
  
    std::cerr << "Cloud after filtering: " << std::endl;
    std::cerr << *cloud_filtered << std::endl;

/*****     Output result of PointCloud in pcd file     *****/
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZI> ("sozai_after_z.pcd", *cloud_filtered, false);

/*****     Publish result of PointCloud2     *****/
    //pub.publish (output_cloud); // For PointCloud output

}


int main(int argc, char **argv)
{
    // Initiate new ROS node named "get_scan_value_3d"
    ros::init(argc, argv, "get_scan_value_3d");
    ros::NodeHandle n;

	tf_listener = new tf::TransformListener();

    // Create a ROS subscriber for input PointCloud
    ros::Subscriber sub = n.subscribe("hokuyo3d/hokuyo_cloud2", 0, cloud_cb);

/*****     Publish result of PointCloud2     *****/
    //pub = n.advertise<sensor_msgs::PointCloud2>("output_road_surface_cloude", 1);

    ROS_INFO("     ************** Start program **************\n");

    ros::spin();
    return 0;
}

