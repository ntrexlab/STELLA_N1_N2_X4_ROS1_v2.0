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

#include <sensor_msgs/MagneticField.h>
#include <std_msgs/Float64.h>
#include <memory>
#include <string>
#include <pthread.h>
#include "MW_serial.hpp"



#define ACC 0x33
#define GYO 0x34
#define DEG 0x35
#define MAG 0x36

#define convertor_g2a 9.80665        // linear_acceleration (g to m/s^2)
#define convertor_d2r (M_PI / 180.0) // angular_velocity (degree to radian)
#define convertor_ut2t 1000000       // magnetic_field (uT to Tesla)
#define convertor_c 1.0              // temperature (celsius)

#define DEG2RAD( a ) ( (a) * (M_PI/180.0f) )
#define COS(a) cos(DEG2RAD(a))
#define SIN(a) sin(DEG2RAD(a))

#define DeviceID 0x01
#define STX 0x02
#define ETX 0x03
#define Command 0xF0


using namespace std;

pid_t pid;
pthread_t tid;
bool publish_tf_;

double linear_acceleration_stddev_;
double angular_velocity_stddev_;
double magnetic_field_stddev_;
double orientation_stddev_;
double ahrs_yaw;

static float acc_value[3] = {
    0,
},
             gyr_value[3] = {
                 0,
},
             deg_value[3] = {
                 0,
},
             mag_value[3] = {
                 0,
};
static bool run_bool = false;

sensor_msgs::Imu imu;

ros::Time current_time,last_time;

class MwAhrsRosDriver 
{
    private:

    ros::Publisher ahrs_pub, odom_pub;
    ros::Publisher imu_data_raw_pub_, imu_data_pub_,imu_mag_pub_,imu_yaw_pub_, imu_pub_;
    ros::Subscriber sub;
    ros::NodeHandle n;

    ros::Timer timer;
   
    std::string parent_frame_id_;
    std::string frame_id_;

    tf::TransformBroadcaster odom_broadcaster; 

    public:

    ~MwAhrsRosDriver();
    MwAhrsRosDriver();

    bool publish_topic();
    
    tf2::Quaternion Euler2Quaternion(float roll, float pitch, float yaw);

    void serial_callback(const ros::TimerEvent& event);

};
