#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

using namespace std;

ros::Subscriber LaserScan_subscriber;

void get_scan_value(const sensor_msgs::LaserScan::ConstPtr& scan);


int main(int argc, char **argv)
{
    //Initiate new ROS node named "get_scan_value"
    ros::init(argc, argv, "get_scan_value");
    ros::NodeHandle n;
    LaserScan_subscriber = n.subscribe<sensor_msgs::LaserScan>("/scan", 10, get_scan_value);

    ROS_INFO("     ************** Start reading scan value **************\n");

    ros::spin();
    return 0;
}


void get_scan_value(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    int vec_size = scan->intensities.size();
    printf("vec_size: %d  ",vec_size);
    printf("Inte: %f  ",scan->intensities[vec_size/2]);
    printf("reng: %f\n",scan->ranges[vec_size/2]);
}
