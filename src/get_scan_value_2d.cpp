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
    const int sensor_pitch_degree = 17;
    const double pi = 3.14159;
    int    laser_sumNumber   = scan->ranges.size();
    double range     = scan->ranges[laser_sumNumber/2];
    double intensity = scan->intensities[laser_sumNumber/2];
    double sensor_hight      = scan->ranges[laser_sumNumber/2] * sin(sensor_pitch_degree * pi /180);
    double l_Robot2MesurLIne = scan->ranges[laser_sumNumber/2] * cos(sensor_pitch_degree * pi /180);
    double degClacForML = asin(scan->ranges[laser_sumNumber/2]/scan->ranges[1]);


    printf("l_sumNum:%d  ", laser_sumNumber);
    printf("s_int:%f  ",intensity); // sentor of intensity
    printf("s_ran:%f  ",range); // sentor of range
    printf("f_ran:%f  ",scan->ranges[1]); // distance measurement line
    printf("h_sen:%f  ", sensor_hight); // sensor hight
    printf("d_R2MLine:%f  ", l_Robot2MesurLIne); // distance between roboto and measurement line
    printf("d_MLine:%f\n", 2 * scan->ranges[1] * cos(degClacForML)); // distance measurement line

}
