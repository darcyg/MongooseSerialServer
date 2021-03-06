#ifndef STATUSPUBLISHER_H
#define STATUSPUBLISHER_H

#include "ros/ros.h"
#include <boost/thread.hpp>
#include <boost/assign/list_of.hpp>
#include <algorithm>
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"
#include "tf/transform_broadcaster.h"
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointField.h>


#include <list>
#include <time.h>
// #include <chrono>

#define PI 3.14159265

namespace xqserial_server
{
typedef struct {
    int status;//小车状态，0表示未初始化，1表示正常，-1表示error
    float power;//电源电压【9 13】v
    float theta;//方位角，【0 360）°

    int encoder_ppr;//车轮1转对应的编码器个数
    int encoder_delta_r;//右轮编码器增量， 个为单位
    int encoder_delta_l;//左轮编码器增量， 个为单位
    int encoder_delta_car;//两车轮中心位移，个为单位

    int omga_r;//右轮转速 个每秒
    int omga_l;//左轮转速 个每秒
    int omga_FR;
    int omga_FL;
    int omga_BL;
    int omga_BR;

    float velocity_x;
    float velocity_y;
    float angular_vel;


    float distance1;//第一个超声模块距离值 单位cm
    float distance2;//第二个超声模块距离值 单位cm
    float distance3;//第三个超声模块距离值 单位cm
    float distance4;//第四个超声模块距离值 单位cm

    float IMU[9];//mpu6050 9轴数据
    float gyro;
    float m_gyro_angular_vel;
    float m_gyro_acc_x;

    unsigned int m_obstacle_state;
    unsigned int time_stamp;//时间戳
    

}UPLOAD_STATUS;

class StatusPublisher
{

public:
    StatusPublisher();

    StatusPublisher(double separation,double radius);

    void Refresh();

    void Update(const char *data, unsigned int len);

    void encoder_IMU_parse(std::vector<unsigned char> data);

    bool crc_check(std::vector<unsigned char> data);

    double get_wheel_separation();

    double get_wheel_radius();

    int get_wheel_ppr();

    int get_status();

    void get_wheel_speed(double speed[2]);

    geometry_msgs::Pose2D get_CarPos2D();

    geometry_msgs::Twist get_CarTwist();

    std_msgs::Float64 get_power();

    nav_msgs::Odometry get_odom();

    UPLOAD_STATUS car_status;
    
    //data type used for compute the sensor data average value.
    std::list<float> ml_position_x;
    std::list<float> ml_position_y;
    std::list<float> ml_gyro_theta;
    std::list<float> ml_gyro_angular_vel;
    std::list<float> ml_encoder_vx;
    std::list<float> ml_encoder_vy;

    float mf_average_position_x;
    float mf_average_position_y;
    float mf_average_gyro_theta;
    float mf_average_gyro_angular_vel;
    float mf_average_encoder_vx;
    float mf_average_encoder_vy;

    //obstacle state from stm32.
    int g_obstacle_state;

private:

    //Wheel separation, wrt the midpoint of the wheel width: meters
    double wheel_separation;

    //Wheel radius (assuming it's the same for the left and right wheels):meters
    double wheel_radius;

    unsigned long package_last_time_stamp;

    geometry_msgs::Pose2D CarPos2D;//小车开始启动原点坐标系
    geometry_msgs::Twist  CarTwist;//小车自身坐标系
    std_msgs::Float64 CarPower;// 小车电池信息
    nav_msgs::Odometry CarOdom;// 小车位置和速度信息

    ros::NodeHandle mNH;
    ros::Publisher mPose2DPub;
    ros::Publisher mTwistPub;
    ros::Publisher mStatusFlagPub;
    ros::Publisher mPowerPub;
    ros::Publisher mOdomPub;
    ros::Publisher pub_barpoint_cloud_;
    ros::Publisher pub_clearpoint_cloud_;

    bool mbUpdated;

    boost::mutex mMutex;
};

} //namespace xqserial_server


#endif // STATUSPUBLISHER_H
