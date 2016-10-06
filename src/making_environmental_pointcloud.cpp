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
        ros::Publisher  disting_cloud_pub;

        tf::TransformListener listener;
        std::string error_msg;
        laser_geometry::LaserProjection projector;

        sensor_msgs::PointCloud2 cloud2;
        sensor_msgs::PointCloud2 disting_cloud2;

        void diagScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in);
};


Making_Envir_Cloud::Making_Envir_Cloud()
{
    diag_scan_sub   = nh.subscribe<sensor_msgs::LaserScan>("/diag_scan", 100, &Making_Envir_Cloud::diagScanCallback, this);
    disting_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/disting_cloud2", 100, false);
    diag_scan_20Hz_pub = nh.advertise<sensor_msgs::LaserScan>("/diag_scan_20Hz", 100, false);
}


void Making_Envir_Cloud::diagScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
    struct timeval s, e;
    ros::Rate loop_rate(20); // 20Hz = 50ms
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_save_cloud (new pcl::PointCloud<pcl::PointXYZI>);

    gettimeofday(&s, NULL);


    /* Transform diagonally_hokuyo_link frame to map frame */
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


    /* Transform LaserScan msg to PointCloud2 msg */
    try{
        projector.transformLaserScanToPointCloud("/map", *scan_in, cloud2, listener);
    }catch(tf::TransformException e){
        ROS_WARN("Could not transform scan to pointcloud! (%s)", e.what());
        return;
    }

    /* Conversion PointCloud2 intensity field name to PointXYZI intensity field name */
    cloud2.fields[3].name = "intensity";
    pcl::fromROSMsg(cloud2, *pcl_cloud);


    /* Partition processing */
    for(int i = 0; i < pcl_cloud->points.size(); i++){
        double normaliz = scan_in->intensities[i] / (-23.4136 * scan_in->ranges[i] * scan_in->ranges[i] - 143.118 * scan_in->ranges[i] + 2811.35);

        if(normaliz >= 1)
            pcl_cloud->points[i].intensity = 1.0;
        else
            pcl_cloud->points[i].intensity = 0.0;
    }


    /* Removing cloud processing */
    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud (pcl_cloud);
    pass.setFilterFieldName ("intensity");
    pass.setFilterLimits (1.0, 1.1); /* remove cloud between 0 and 0.1 intensity value */
    pass.setFilterLimitsNegative (false);
    pass.filter (*pcl_save_cloud);

    /* Saving processing */
    string file_path = "/home/kenta/pcd/making_envir_cloud/";
    string file_name;
    static int i = 1;
    char buf[10];

    sprintf(buf, "%d", i);
    file_name.append(file_path);
    file_name.append(buf);
    file_name.append(".pcd");
    //cout << file_name << endl;

    //pcl::io::savePCDFileASCII (file_name, *pcl_save_cloud);
    i++;


    toROSMsg (*pcl_save_cloud, disting_cloud2);
    disting_cloud_pub.publish(disting_cloud2);
    diag_scan_20Hz_pub.publish(scan_in);
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

