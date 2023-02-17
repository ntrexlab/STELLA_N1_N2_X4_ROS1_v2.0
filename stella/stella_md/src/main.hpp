#include <thread>
#include <ros/ros.h>
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <sensor_msgs/Imu.h>
#include <vector>
#include <sstream>
#include <iostream>
#include <stdlib.h>
#include <tf2/LinearMath/Quaternion.h>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <string.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float64.h>

using namespace std;

sensor_msgs::Imu imu;

ros::Time current_time, last_time;

double goal_linear_velocity_;
double goal_angular_velocity_;

int left_encoder_prev =0, right_encoder_prev =0;

double ahrs_yaw, delta_th=0.0,delta_s=0.0,delta_x=0.0,delta_y=0.0,x=0.0,y=0.0,th=0.0,delta_left = 0,delta_right = 0;


class stellaN1_node 
{
    private:
    ros::Publisher chatter_pub;
    ros::Subscriber ahrs_sub;
    ros::Subscriber sub;

    ros::NodeHandle n;

    ros::Timer timer;
   
    std::thread* thread_read_AHRS;
    std::thread* thread_pub_odm;

    tf::TransformBroadcaster odom_broadcaster;
 
    public:

    ~stellaN1_node();
    stellaN1_node();

    void ahrs_yaw_data_callback(const std_msgs::Float64::ConstPtr& ahrs_msg);
    void command_velocity_callback(const geometry_msgs::Twist::ConstPtr& cmd_msg);
    void serial_callback(const ros::TimerEvent& event);
    bool update_odometry();

};
