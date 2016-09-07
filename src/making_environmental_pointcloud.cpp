#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
// Setting to transform LaserScan into PointCloud
#include <laser_geometry/laser_geometry.h>
// Setting to PCL library
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>

using namespace std;


class Making_Envir_Cloud
{
    public:
        Making_Envir_Cloud(); // Constructor

    private:
        ros::NodeHandle n;
        ros::Subscriber diag_scan_sub;
        ros::Publisher  point_cloud_pub;

        tf::TransformListener listener;
        std::string error_msg;
        laser_geometry::LaserProjection projector;

        sensor_msgs::PointCloud2 cloud;

        void diagScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in);

};


Making_Envir_Cloud::Making_Envir_Cloud() // Constructor
{
    diag_scan_sub   = n.subscribe<sensor_msgs::LaserScan>("/diag_scan", 100, &Making_Envir_Cloud::diagScanCallback, this);
    point_cloud_pub = n.advertise<sensor_msgs::PointCloud2>("/cloud", 100, false);
}


void Making_Envir_Cloud::diagScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr save_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr conv_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    if(listener.waitForTransform(
                scan_in->header.frame_id, 
                "/map",
                scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
                ros::Duration(1.0),
                ros::Duration(0.01),
                &error_msg ) == 0){
        cout <<"Failure in waitForTransform" << endl;
		ROS_WARN("%s", error_msg.c_str());
        return;
    }

    try{
        projector.transformLaserScanToPointCloud("/map", *scan_in, cloud, listener);
    }catch(tf::TransformException e){
		ROS_WARN("Could not transform scan to pointcloud! (%s)", e.what());
		return;
    }

	//Conversion PointCloud2 intensity field name to PointXYZI intensity field name.
	cloud.fields[3].name = "intensity";
	pcl::fromROSMsg(cloud, *conv_cloud);

    //cout << save_cloud->points.size();
    //cout << conv_cloud->points.size() << endl;
    for (int i = 0; i < conv_cloud->points.size(); ++i)
    {
        cout << "i = " << i << endl;
        save_cloud->points[save_cloud->points.size() + i].x = conv_cloud->points[i].x;
        save_cloud->points[save_cloud->points.size() + i].y = conv_cloud->points[i].y;
        save_cloud->points[save_cloud->points.size() + i].z = conv_cloud->points[i].z;
        save_cloud->points[save_cloud->points.size() + i].intensity = conv_cloud->points[i].intensity;
    }


    //pcl::io::savePCDFileASCII ("making_envir_cloud.pcd", *save_cloud);


    point_cloud_pub.publish(cloud);
    cout << "Success in publishing " << endl;

}


int main(int argc, char **argv)
{
    //Initiate new ROS node named "making_envir_cloud"
    ros::init(argc, argv, "making_envir_cloud");

    ROS_INFO("\n************** Start this program **************\n");
    Making_Envir_Cloud making_envir_cloud;

    ros::spin();
    return 0;

}

