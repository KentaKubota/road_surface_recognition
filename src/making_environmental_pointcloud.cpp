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
    ros::Rate loop_rate(10); // 10 = 100ms
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud (new pcl::PointCloud<pcl::PointXYZI>);

    gettimeofday(&s, NULL);


    /* Transform diagonally_hokuyo_link frame to map frame */
    if(listener.waitForTransform(
                scan_in->header.frame_id, 
                "/map",
                scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
                ros::Duration(1.0),
                ros::Duration(0.01),
                &error_msg ) == 0){
        std::cout <<"Failure in waitForTransform" << std::endl;
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


    /* Variable for rms */
    double a, b; //for rms error function
    double sum_z, sum_y, sum_y2, sum_yz;
    const int N = 100;
    sum_z = sum_y = sum_y2 = sum_yz = 0;

    for(int i = 200; i <= 299 ; i++){
        sum_y += pcl_cloud->points[i].y;
        sum_z += pcl_cloud->points[i].z;
        sum_yz += pcl_cloud->points[i].y * pcl_cloud->points[i].z;
        sum_y2 += pcl_cloud->points[i].y * pcl_cloud->points[i].y;
    }
    a = (N * sum_yz - sum_y * sum_z) / (N * sum_y2 - pow(sum_y,2));
    b = (sum_y2 * sum_z - sum_yz * sum_y) / (N * sum_y2 - pow(sum_y,2));

    //if(a <= -0.025 || a >= 0.025)
    //    return ;

    /* Detect low level processing and distinguished cloud processing */
    for(int i = 0; i < pcl_cloud->points.size(); i++){
        double normaliz = scan_in->intensities[i] / (48.2143 * scan_in->ranges[i] * scan_in->ranges[i] - 840.393 * scan_in->ranges[i] + 4251.14+300+300);

        //if(pcl_cloud->points[i].z >= a * pcl_cloud->points[i].y + b + 0.038){
        //    pcl_cloud->points[i].intensity = 100.0;
        //}else{
            if(normaliz >= 1)
                pcl_cloud->points[i].intensity = 100.0;
            else
                pcl_cloud->points[i].intensity = 0.1;
        //}
    }


    /* Saving processing */
    std::string file_path = "/home/kenta/pcd/making_envir_cloud/";
    std::string file_name;
    static int i = 1;
    char buf[10];

    sprintf(buf, "%d", i);
    file_name.append(file_path);
    file_name.append(buf);
    file_name.append(".pcd");

    pcl::io::savePCDFileASCII (file_name, *pcl_cloud);
    i++;


    toROSMsg (*pcl_cloud, disting_cloud2);
    disting_cloud_pub.publish(disting_cloud2);
    diag_scan_20Hz_pub.publish(scan_in);
    std::cout << "Success in publishing   ";

    loop_rate.sleep();
    gettimeofday(&e, NULL);
    std::cout << "Compute time: " <<  std::setw(9) << (e.tv_sec - s.tv_sec) + (e.tv_usec - s.tv_usec)*1.0E-6 << "s" << std::endl;
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

