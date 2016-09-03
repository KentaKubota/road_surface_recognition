#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
// Setting to transform LaserScan into pointcloud
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>

using namespace std;

ros::Subscriber diag_scan_subscriber;
ros::Publisher  point_cloud_publisher;


//この関数の中でLaserScanを読み込み、mapフレーム座標のPointcloudに変換、
//点群の逐次保存を行う
void making_envir_cloud(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
    laser_geometry::LaserProjection projector;
    tf::TransformListener listener;

	// Transform pointcloud from LIDAR fixed link to base_link
    //if(listener.waitForTransform("/map", scan_in->header.frame_id, ros::Time(),ros::Duration(1.0)) == 0){
    //    cout <<"return " << endl;
    //    return ;
    //}


    if(listener.waitForTransform(scan_in->header.frame_id, "/map"
                ,scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment)
                ,ros::Duration(1.0)) == 0){
        cout <<"Failure in waitForTransform" << endl;
        return;
    }

    cout << "Success in waitForTransform " << endl;

    sensor_msgs::PointCloud cloud;
    projector.transformLaserScanToPointCloud("/map", *scan_in, cloud, listener);
    point_cloud_publisher.publish(cloud);

}




int main(int argc, char **argv)
{
    //Initiate new ROS node named "making_envir_cloud"
    ros::Time::init();
    ros::init(argc, argv, "making_envir_cloud");
    ros::NodeHandle n;

    ROS_INFO("\n************** Start this program **************\n");
    diag_scan_subscriber = n.subscribe<sensor_msgs::LaserScan>("/diag_scan", 10, making_envir_cloud);
    point_cloud_publisher = n.advertise<sensor_msgs::PointCloud> ("/cloud", 100, false);

    ros::spin();
    return 0;
}

