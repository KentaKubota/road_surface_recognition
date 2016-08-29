#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
// for transform LaserScan into pointcloud
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>

using namespace std;

ros::Subscriber diag_scan_subscriber;
laser_geometry::LaserProjection projector_;
tf::TransformListener listener_;


void making_envir_cloud(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
  if(!listener_.waitForTransform(scan_in->header.frame_id,"/map",scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
        ros::Duration(1.0))){
     return;
  }

  sensor_msgs::PointCloud cloud;
  projector_.transformLaserScanToPointCloud("/map",*scan_in,cloud,listener_);

}


int main(int argc, char **argv)
{
    //Initiate new ROS node named "making_envir_cloud"
    ros::init(argc, argv, "making_envir_cloud");
    ros::NodeHandle n;

    ROS_INFO("     ************** Start reading scan value of 2DURG **************\n");
    diag_scan_subscriber = n.subscribe<sensor_msgs::LaserScan>("/diag_scan", 10, making_envir_cloud);

    ros::spin();
    return 0;
}


