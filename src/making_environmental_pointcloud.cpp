#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <sys/time.h>
/* Setting for transform LaserScan into PointCloud */
#include <laser_geometry/laser_geometry.h>
/* Setting for PCL library */
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
/* Setting for convertPointCloud2ToPointCloud */
#include <sensor_msgs/point_cloud_conversion.h>

using namespace std;


class Making_Envir_Cloud
{
    public:
        /* Constructor */
        Making_Envir_Cloud();

    private:
        ros::NodeHandle nh;
        ros::Subscriber diag_scan_sub;
        ros::Publisher  diag_scan_20Hz_pub;

        sensor_msgs::LaserScan lawn_scan;

        void diagScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in);
};


Making_Envir_Cloud::Making_Envir_Cloud()
{
    diag_scan_sub   = nh.subscribe<sensor_msgs::LaserScan>("/diag_scan", 100, &Making_Envir_Cloud::diagScanCallback, this);
    diag_scan_20Hz_pub = nh.advertise<sensor_msgs::LaserScan>("/diag_scan_20Hz", 100, false);
}


void Making_Envir_Cloud::diagScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
    struct timeval s, e;
    ros::Rate loop_rate(20); // 20Hz = 50ms

    gettimeofday(&s, NULL);

    lawn_scan.header.stamp = scan_in->header.stamp;
    lawn_scan.header.frame_id = scan_in->header.frame_id;
    lawn_scan.angle_min = scan_in->angle_min;
    lawn_scan.angle_max = scan_in->angle_max;
    lawn_scan.angle_increment = scan_in->angle_increment;
    lawn_scan.time_increment = scan_in->time_increment;
    lawn_scan.range_min = scan_in->range_min;
    lawn_scan.range_max = scan_in->range_max;
    lawn_scan.ranges.resize(scan_in->ranges.size());
    lawn_scan.intensities.resize(scan_in->ranges.size());

    /* Partition processing */
    for(int i = 0; i < scan_in->ranges.size(); i++){
        double normaliz = scan_in->intensities[i] / (-23.4136 * scan_in->ranges[i] * scan_in->ranges[i] - 143.118 * scan_in->ranges[i] + 2811.35);

        if(normaliz >= 1){
            lawn_scan.ranges[i] = scan_in->ranges[i];
            lawn_scan.intensities[i] = scan_in->intensities[i];
        }
        else{
            lawn_scan.ranges[i] = 0;
            lawn_scan.intensities[i] = 0;
        }
    }

    diag_scan_20Hz_pub.publish(lawn_scan);
    cout << "Success in publishing   ";

    loop_rate.sleep();
    gettimeofday(&e, NULL);
    cout << "Compute time: " << (e.tv_sec - s.tv_sec) + (e.tv_usec - s.tv_usec)*1.0E-6 << "s" << endl;
}


int main(int argc, char **argv)
{
    /* Initiate new ROS node named "making_envir_cloud" */
    ros::init(argc, argv, "making_envir_cloud");

    ROS_INFO("\n************** Start this program **************\n");
    Making_Envir_Cloud making_envir_cloud;

    ros::spin();
    return 0;
}

