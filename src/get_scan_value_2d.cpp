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

    ROS_INFO("     ************** Start reading scan value of 2DURG **************\n");
    LaserScan_subscriber = n.subscribe<sensor_msgs::LaserScan>("/diag_scan", 10, get_scan_value);

    ros::spin();
    return 0;
}


void get_scan_value(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    const int sensor_pitch_degree 17;
    int laser_num = scan->intensities.size();

    printf("laser sumNumber: %d  ", laser_num);
    printf("intensity: %f  ",scan->intensities[laser_num/2]);
    printf("range: %f  ",scan->ranges[laser_num/2]);
    printf("sensor hight: %f  ",scan->ranges[laser_num/2] * sin(sensor_pitch_degree));
    printf("distanceRobot2Line: %f\n",scan->ranges[laser_num/2] * cos(sensor_pitch_degree));

}
