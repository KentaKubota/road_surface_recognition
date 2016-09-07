#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
// Setting to transform LaserScan into PointCloud
#include <laser_geometry/laser_geometry.h>

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
        laser_geometry::LaserProjection projector;

        sensor_msgs::PointCloud cloud;

        void diagScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in);

};


Making_Envir_Cloud::Making_Envir_Cloud() // Constructor
{
    diag_scan_sub   = n.subscribe<sensor_msgs::LaserScan>("/diag_scan", 10, &Making_Envir_Cloud::diagScanCallback, this);
    point_cloud_pub = n.advertise<sensor_msgs::PointCloud>("/cloud", 100, false);
}


void Making_Envir_Cloud::diagScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{

    if(listener.waitForTransform(
                scan_in->header.frame_id, 
                "/map",
                scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
                ros::Duration(1.0) ) == 0){
        cout <<"Failure in waitForTransform" << endl;
        return;
    }
    cout << "Success in waitForTransform " << endl;

    projector.transformLaserScanToPointCloud("/map", *scan_in, cloud, listener);
    point_cloud_pub.publish(cloud);

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

