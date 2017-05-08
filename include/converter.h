#include <csignal>
#include <map>
#include <limits>
#include <fstream>
#include <iomanip>
#include <vector>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/tf.h"
#include "tf/transform_listener.h"
#include <tf/transform_broadcaster.h>

#include <rosbag/bag.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#include <nav_msgs/Odometry.h>

class Converter{
    std::ifstream ifs;
    std::ofstream ofs;
    rosbag::Bag bag;

    geometry_msgs::TransformStamped tf_odom_robot_msg;
    geometry_msgs::TransformStamped tf_laser_robot_msg;

    tf::tfMessage tf2_msg;

    nav_msgs::Odometry pose_msg;

    sensor_msgs::LaserScan laser_msg;

    ros::Time stamp;
    double rate;

    std::string tf_topic, robot_link, odom_link, odom_robot_link, true_odom_link, ROBOTLASER1_link;

    std::map<std::string, std::string> links;

public:
    Converter(char* in, char* out, char* params);
    ~Converter();

    void convert();

    void handleOdom();
    void handleLaserMsg();
    void handleRobotLaserMsg();

    void readRangesInfo();

    void incrementStamp();
};