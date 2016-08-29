#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
// for transform LaserScan into pointcloud
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>

using namespace std;

ros::Subscriber diag_scan_subscriber;
ros::Publisher point_cloud_publisher_;
laser_geometry::LaserProjection projector_;
tf::TransformListener tfListener_;

//この関数の中でLaserScanを読み込み、mapフレーム座標のPointcloudに変換、
//点群の逐次保存を行う
void making_envir_coud(const sensor_msgs::LaserScan::ConstPtr& scan);

int main(int argc, char **argv)
{
    //Initiate new ROS node named "making_envir_coud"
    ros::init(argc, argv, "making_envir_coud");
    ros::NodeHandle n;

    //ROS_INFO("     ************** Start reading scan value of 2DURG **************\n");
    diag_scan_subscriber = n.subscribe<sensor_msgs::LaserScan>("/diag_scan", 10, making_envir_coud);
    point_cloud_publisher_ = n.advertise<sensor_msgs::PointCloud2> ("/cloud", 100, false);
    tfListener_.setExtrapolationLimit(ros::Duration(0.1));

    ros::spin();
    return 0;
}


void making_envir_coud(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    sensor_msgs::PointCloud2 cloud;
    projector_.transformLaserScanToPointCloud("/map", *scan, cloud, tfListener_);
    point_cloud_publisher_.publish(cloud);
}
