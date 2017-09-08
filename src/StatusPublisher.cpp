#include "StatusPublisher.h"
#include "AsyncSerial.h"
#include <memory.h>
#include <math.h>
#include <stdlib.h>

using namespace std;

#define DISABLE 0
#define ENABLE 1

//serial package data.
//串口协议参数定义
#define PACKAGE_BEGIN 255
#define COMMOND_BIT 249
#define PACKAGE_END 237
#define PACKAGE_LEN 50
#define CRC_BIT 48
#define Pi 3.1415926

namespace xqserial_server
{
typedef sensor_msgs::PointCloud2 PointCloud;
bool detect_pack = false;
std::vector<unsigned char> vdataBuffer;
bool start_parse = false;
int receive_data_size;

StatusPublisher::StatusPublisher()
{
    mbUpdated = false;
    wheel_separation = 0.37;
    wheel_radius = 0.06;

    CarPos2D.x = 0.0;
    CarPos2D.y = 0.0;
    CarPos2D.theta = 0.0;

    CarTwist.linear.x = 0.0;
    CarTwist.linear.y = 0.0;
    CarTwist.linear.z = 0.0;
    CarTwist.angular.x = 0.0;
    CarTwist.angular.y = 0.0;
    CarTwist.angular.z = 0.0;

    CarPower.data = 0.0;

    int i = 0;
    int *status;
    status = (int *)&car_status;
    for (i = 0; i < 23; i++)
    {
        status[i] = 0;
    }
    car_status.encoder_ppr = 4 * 12 * 64;

    mPose2DPub = mNH.advertise<geometry_msgs::Pose2D>("xqserial_server/Pose2D", 1, true);
    mStatusFlagPub = mNH.advertise<std_msgs::Int32>("xqserial_server/StatusFlag", 1, true);
    mTwistPub = mNH.advertise<geometry_msgs::Twist>("xqserial_server/Twist", 1, true);
    mPowerPub = mNH.advertise<std_msgs::Float64>("xqserial_server/Power", 1, true);
    mOdomPub = mNH.advertise<nav_msgs::Odometry>("xqserial_server/Odom", 1, true);
    pub_barpoint_cloud_ = mNH.advertise<PointCloud>("kinect/barpoints", 1, true);
    pub_clearpoint_cloud_ = mNH.advertise<PointCloud>("kinect/clearpoints", 1, true);

    /* static tf::TransformBroadcaster br;
   tf::Quaternion q;
   tf::Transform transform;
   transform.setOrigin( tf::Vector3(0.0, 0.0, 0.13) );//摄像头距离地面高度13cm
   q.setRPY(0, 0, 0);
   transform.setRotation(q);
   br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_footprint", "base_link"));
   */
}

StatusPublisher::StatusPublisher(double separation, double radius)
{
    new (this) StatusPublisher();
    wheel_separation = separation;
    wheel_radius = radius;
}

void StatusPublisher::Update(const char data[], unsigned int len)
{
    // if(len <1) return;
    // static char data2[1024];
    // static int len2=0;
    boost::mutex::scoped_lock lock(mMutex);

    int i = 0, j = 0;
    int *receive_byte;
    static unsigned int last_data[2] = {0x00, 0x00};
    static unsigned char last_str[2] = {0x00, 0x00};
    static unsigned char new_packed_ctr = DISABLE; //ENABLE表示新包开始，DISABLE 表示上一个包还未处理完；
    static int new_packed_ok_len = 0;              //包的理论长度
    static int new_packed_len = 0;                 //包的实际长度
    static unsigned char cmd_string_buf[17];
    unsigned char current_str = 0x00;
    const int cmd_string_max_size = 512;
    receive_byte = (int *)&car_status;
    //int ii=0;
    //boost::mutex::scoped_lock lock(mMutex);

    if (len < 119)
    {
        //  std::cout<<"len0:"<<len<<std::endl;
        current_str = data[0];
        //    std::cout<<(unsigned int)current_str<<std::endl;
    }

    for (i = 0; i < len; i++)
    {
        current_str = data[i];
        // unsigned int temp=(unsigned int)current_str;
        // std::cout<<temp<<std::endl;

        //判断是否有新包头
        if (last_str[0] == PACKAGE_END && last_str[1] == PACKAGE_BEGIN && current_str == COMMOND_BIT) //包头 205 235 215
        {
            new_packed_ctr = ENABLE;
            new_packed_ok_len = 0;
            new_packed_len = new_packed_ok_len;
            last_str[0] = last_str[1]; //保存最后两个字符，用来确定包头
            last_str[1] = current_str;
        }
        last_str[0] = last_str[1]; //保存最后两个字符，用来确定包头
        last_str[1] = current_str;

        if (new_packed_ctr == ENABLE)
        {

            //获取包长度
            new_packed_ok_len = current_str;
            if (new_packed_ok_len > cmd_string_max_size)
                new_packed_ok_len = cmd_string_max_size; //包内容最大长度有限制
        }
        if (last_data[0] = PACKAGE_END && current_str == PACKAGE_BEGIN)
        {
            start_parse = true;
            receive_data_size = 0;
        }
        last_data[0] = current_str;

        if (start_parse)
        {
            vdataBuffer.push_back(current_str);
            if (vdataBuffer.size() == PACKAGE_LEN)
            {
                //Compute time cost of parse data from sensors.
                clock_t start = clock();
                STM32_Encoder_IMU_Parse(vdataBuffer);
                clock_t stop = clock();
                double elapsed = (double)(stop - start) * 1000.0 / CLOCKS_PER_SEC;
                // ROS_INFO("Time cost of data parse:[%f]", elapsed);

                mbUpdated = true;
                start_parse = false;

                // Raw data from serial.
                for (int j = 0; j < PACKAGE_LEN; ++j)
                {
                    // std::cout << "data:" << (int)vdataBuffer[j] << std::endl;
                    // // 输出异常值
                    //if((int)vdataBuffer[3] !=0 || (int)vdataBuffer[4] !=0)
                    //{
                    //  ROS_INFO("origin gyro theta:[%d]" , (int)vdataBuffer[35]);
                    //  ROS_INFO("theta:[%d]", (int)vdataBuffer[36]);
                    //  ROS_INFO("theta:[%d]", (int)vdataBuffer[37]);
                    //}
                }
                vdataBuffer.clear();
            }
        }
    }
    return;
}

void StatusPublisher::STM32_Encoder_IMU_Parse(std::vector<unsigned char> data)
{
    unsigned char i = 0;
    unsigned char command = 0;
    unsigned long FR_spin_speed_temp = 0;
    unsigned long FL_spin_speed_temp = 0;
    unsigned long BL_spin_speed_temp = 0;
    unsigned long BR_spin_speed_temp = 0;

    //from encoder.
    unsigned long speed_x_temp = 0;
    unsigned long speed_y_temp = 0;
    unsigned long angular_vel_temp = 0;

    //from IMU.
    unsigned long imu_acc_x_temp = 0;
    unsigned long imu_acc_y_temp = 0;
    unsigned long imu_acc_z_temp = 0;

    unsigned long imu_angular_vel_x_temp = 0;
    unsigned long imu_angular_vel_y_temp = 0;
    unsigned long imu_angular_vel_z_temp = 0;

    unsigned long imu_angular_x_temp = 0;
    unsigned long imu_angular_y_temp = 0;
    unsigned long imu_angular_z_temp = 0;

    //package time stamp.
    unsigned long long time_stamp_temp = 0;

    //Gryo data.
    unsigned long gyro_data = 0;
    int gyro_state = 0;

    unsigned long gyro_angular_vel = 0;
    int gyro_angular_vel_state = 0;

    unsigned long gyro_acc_x = 0;
    int gyro_acc_x_state = 0;

    //from encoder.
    float speed_x = 0.0;
    float speed_y = 0.0;
    float angular_vel = 0.0;

    int speed_x_check = 0;
    int speed_y_check = 0;
    int angular_vel_check = 0;

    //from IMU.
    float imu_acc_x = 0.0;
    float imu_acc_y = 0.0;
    float imu_acc_z = 0.0;

    float imu_angular_vel_x = 0.0;
    float imu_angular_vel_y = 0.0;
    float imu_angular_vel_z = 0.0;

    float imu_angular_x = 0.0;
    float imu_angular_y = 0.0;
    float imu_angular_z = 0.0;

    unsigned int obstacle_state = 0;

    //package time stamp.
    unsigned long time_stamp_sensors = 0;

    float FR_spin_speed = 0.0;
    float FL_spin_speed = 0.0;
    float BL_spin_speed = 0.0;
    float BR_spin_speed = 0.0;

    bool crc_ok = false;
    crc_ok = crc_check(data);

    if (crc_ok)
    {
        command = data[1];

        // std::cout << "crc OK:" << __LINE__ << std::endl;
        switch (command)
        {
        case 0x01:
            break;

        case 0xF9:
            //parse Vx.
            speed_x_temp |= data[3];
            speed_x_temp = (speed_x_temp << 8);
            speed_x_temp |= data[4];
            speed_x = (float)(speed_x_temp / 100.0);
            speed_x_check = (int)data[5];

            if (speed_x_check == 1)
            {
                car_status.velocity_x = -speed_x;
            }
            else
            {
                car_status.velocity_x = speed_x;
            }

            //计算数据的平均值
            if (ml_encoder_vx.size() < 10)
            {
                ml_encoder_vx.push_back(car_status.velocity_x);
            }
            else
            {
                ml_encoder_vx.pop_front();
                ml_encoder_vx.push_back(car_status.velocity_x);
            }

            if (ml_encoder_vx.size() == 10)
            {
                mf_average_encoder_vx = 0;
                for (std::list<float>::iterator it = ml_encoder_vx.begin(); it != ml_encoder_vx.end(); ++it)
                {
                    mf_average_encoder_vx += *it;
                    // std::cout << "Velocity x:" << *it << std::endl;
                }
                mf_average_encoder_vx = mf_average_encoder_vx / 10;
                // std::cout << "average vx data:" << mf_average_encoder_vx << std::endl;
            }

            if (abs(car_status.velocity_x) > 5 * abs(mf_average_encoder_vx) || abs(car_status.velocity_x) < abs(mf_average_encoder_vx) / 5)
            {
                car_status.velocity_x = mf_average_encoder_vx;
                // std::cout << "ERROR data vel x:" << car_status.velocity_x << std::endl;
            }

            //parse Vy.
            speed_y_temp |= data[6];
            speed_y_temp = (speed_y_temp << 8);
            speed_y_temp |= data[7];
            speed_y = (float)(speed_y_temp / 100.0);
            speed_y_check = data[8];
            // std::cout << "y符号判断：" << (int)data[8] << std::endl;

            //小车速度方向判断和到move_base的坐标系转换。
            if (speed_y_check == 1)
            {
                car_status.velocity_y = -speed_y;
                // std::cout <<  "y: " << car_status.velocity_y << std::endl;
            }
            else
            {
                car_status.velocity_y = speed_y;
                // std::cout <<  "y: " << car_status.velocity_y << std::endl;
            }

            //计算vy数据的平均值
            if (ml_encoder_vy.size() < 10)
            {
                ml_encoder_vy.push_back(car_status.velocity_y);
            }
            else
            {
                ml_encoder_vy.pop_front();
                ml_encoder_vy.push_back(car_status.velocity_y);
            }

            if (ml_encoder_vy.size() == 10)
            {
                mf_average_encoder_vy = 0;
                for (std::list<float>::iterator it = ml_encoder_vy.begin(); it != ml_encoder_vy.end(); ++it)
                {
                    mf_average_encoder_vy += *it;
                    // std::cout << "Velocity y:" << *it << std::endl;
                }
                mf_average_encoder_vy = mf_average_encoder_vy / 10;
                // std::cout << "average vy data:" << mf_average_encoder_vy << std::endl;
            }

            if (abs(car_status.velocity_y) > 5 * abs(mf_average_encoder_vy) || abs(car_status.velocity_x) < abs(mf_average_encoder_vy) / 5)
            {
                car_status.velocity_y = mf_average_encoder_vy;
                // std::cout << "ERROR data vel y:" << car_status.velocity_y << std::endl;
            }

            //parse W.
            angular_vel_temp |= data[9];
            angular_vel_temp = (angular_vel_temp << 8);
            angular_vel_temp |= data[10];
            angular_vel = (float)(angular_vel_temp / 100.0);
            angular_vel_check = data[11];
            // std::cout << "w符号判断：" << (int)data[11] << std::endl;

            if (angular_vel_check == 1)
            {
                car_status.angular_vel = -angular_vel;
            }
            else
            {
                car_status.angular_vel = angular_vel;
            }

            //parse IMU acc_x.
            imu_acc_x_temp |= data[12];
            imu_acc_x_temp = (imu_acc_x_temp << 8);
            imu_acc_x_temp |= data[13];
            imu_acc_x = (float)(imu_acc_x_temp / 100.0);
            car_status.IMU[0] = imu_acc_x;

            //parse IMU acc_y.
            imu_acc_y_temp |= data[14];
            imu_acc_y_temp = (imu_acc_y_temp << 8);
            imu_acc_y_temp |= data[15];
            imu_acc_y = (float)(imu_acc_y_temp / 100.0);
            car_status.IMU[1] = imu_acc_y;

            //parse IMU acc_z.
            imu_acc_z_temp |= data[16];
            imu_acc_z_temp = (imu_acc_z_temp << 8);
            imu_acc_z_temp |= data[17];
            imu_acc_z = (float)(imu_acc_z_temp / 100.0);
            car_status.IMU[2] = imu_acc_z;

            //parse IMU angular_vel_x.
            imu_angular_vel_x_temp |= data[18];
            imu_angular_vel_x_temp = (imu_angular_vel_x_temp << 8);
            imu_angular_vel_x_temp |= data[19];
            imu_angular_vel_x = (float)(imu_angular_vel_x_temp / 100.0);
            car_status.IMU[3] = imu_angular_vel_x;

            //parse IMU angular_vel_y.
            imu_angular_vel_y_temp |= data[20];
            imu_angular_vel_y_temp = (imu_angular_vel_y_temp << 8);
            imu_angular_vel_y_temp |= data[21];
            imu_angular_vel_y = (float)(imu_angular_vel_y_temp / 100.0);
            car_status.IMU[4] = imu_angular_vel_y;

            //parse IMU angular_vel_z.
            imu_angular_vel_z_temp |= data[22];
            imu_angular_vel_z_temp = (imu_angular_vel_z_temp << 8);
            imu_angular_vel_z_temp |= data[23];
            imu_angular_vel_z = (float)(imu_angular_vel_z_temp / 100.0);
            car_status.IMU[5] = imu_angular_vel_z;

            //parse IMU angular_x.
            imu_angular_x_temp |= data[24];
            imu_angular_x_temp = (imu_angular_x_temp << 8);
            imu_angular_x_temp |= data[25];
            imu_angular_x = (float)(imu_angular_x_temp / 100.0);
            car_status.IMU[6] = imu_angular_x;

            //parse angular_y.
            imu_angular_y_temp |= data[26];
            imu_angular_y_temp = (imu_angular_y_temp << 8);
            imu_angular_y_temp |= data[27];
            imu_angular_y = (float)(imu_angular_y_temp / 100.0);
            car_status.IMU[7] = imu_angular_y;

            //parse angular_z.
            imu_angular_z_temp |= data[28];
            imu_angular_z_temp = (imu_angular_z_temp << 8);
            imu_angular_z_temp |= data[29];
            imu_angular_z = (float)(imu_angular_z_temp / 100.0);
            car_status.IMU[8] = imu_angular_z;

            //超声波状态返回.
            //1 表示有障碍物，0表示没有障碍物。
            obstacle_state = (unsigned int)data[30];

            //use global variable to control command send action.
            g_obstacle_state = obstacle_state;
            // std::cout << "obstacle state in parse:" << g_obstacle_state << std::endl;

            //parse time stamp.
            time_stamp_temp |= data[31];
            time_stamp_temp = (time_stamp_temp << 24);
            time_stamp_temp |= data[32];
            time_stamp_temp = (time_stamp_temp << 16);
            time_stamp_temp |= data[33];
            time_stamp_temp = (time_stamp_temp << 8);
            time_stamp_temp |= data[34];
            time_stamp_sensors = (unsigned long)(time_stamp_temp);
            car_status.time_stamp = time_stamp_sensors;

            //parse gyro data.
            gyro_data |= data[35];
            gyro_data = (gyro_data << 8);
            gyro_data |= data[36];
            gyro_state = data[37];

            if (gyro_state == 0)
            {
                car_status.gyro = gyro_data / 100;
                //std::cout << "gyro data:" << car_status.gyro << std::endl;
            }
            else
            {
                car_status.gyro = -((float)gyro_data / 100);
                //std::cout << "gyro data:" << car_status.gyro << std::endl;
            }

            gyro_angular_vel |= data[38];
            gyro_angular_vel = (gyro_angular_vel << 8);
            gyro_angular_vel |= data[39];
            gyro_angular_vel_state = data[40];
            if (gyro_angular_vel_state == 0)
            {
                car_status.m_gyro_angular_vel = (float)gyro_angular_vel / 100;
                //   std::cout << "angular vel from gyro:" << car_status.m_gyro_angular_vel << std::endl;
            }
            else
            {
                car_status.m_gyro_angular_vel = -((float)gyro_angular_vel / 100);
                //   std::cout << "angular vel from gyro:" << car_status.m_gyro_angular_vel << std::endl;
            }

            //计算gyro 角速度数据的平均值
            if (ml_gyro_angular_vel.size() < 10)
            {
                ml_gyro_angular_vel.push_back(car_status.m_gyro_angular_vel);
            }
            else
            {
                ml_gyro_angular_vel.pop_front();
                ml_gyro_angular_vel.push_back(car_status.m_gyro_angular_vel);
            }

            if (ml_gyro_angular_vel.size() == 10)
            {
                mf_average_gyro_angular_vel = 0;
                for (std::list<float>::iterator it = ml_gyro_angular_vel.begin(); it != ml_gyro_angular_vel.end(); ++it)
                {
                    mf_average_gyro_angular_vel += *it;
                }
                mf_average_gyro_angular_vel = mf_average_gyro_angular_vel / 10;
            }

            if (abs(car_status.m_gyro_angular_vel) > 5 * abs(mf_average_gyro_angular_vel) || abs(car_status.m_gyro_angular_vel) < abs(mf_average_gyro_angular_vel) / 5)
            {
                car_status.m_gyro_angular_vel = mf_average_gyro_angular_vel;
            }

            // std::cout << "imu_acc_x:" << imu_acc_x << std::endl;
            // std::cout << "speed_x:" << car_status.velocity_x << std::endl;
            // std::cout << "speed_y:" << car_status.velocity_y << std::endl;
            // std::cout << "angular_vel:" << angular_vel << std::endl
            //           << std::endl;
            // std::cout << "imu_acc_y:" << imu_acc_y << std::endl;
            // std::cout << "imu_acc_z:" << imu_acc_z << std::endl
            //           << std::endl;
            // std::cout << "imu_angular_vel_x:" << imu_angular_vel_x << std::endl;
            // std::cout << "imu_angular_vel_y:" << imu_angular_vel_y << std::endl;
            // std::cout << "imu_angular_vel_z:" << imu_angular_vel_z << std::endl
            //           << std::endl;
            // std::cout << "imu_angular_x:" << imu_angular_x << std::endl;
            // std::cout << "imu_angular_y:" << imu_angular_y << std::endl;
            // std::cout << "imu_angular_z:" << imu_angular_z << std::endl
            //           << std::endl;

            // std::cout << "time_stamp:" << time_stamp_sensors << std::endl
            //           << std::endl;
            // std::cout << "Gyro angular:" << car_status.gyro << std::endl;
            // std::cout << "Gyro angular vel:" << car_status.m_gyro_angular_vel << std::endl
            //   << std::endl;
            break;

        default:
            break;
        }
    }
}

bool StatusPublisher::crc_check(std::vector<unsigned char> data)
{
    unsigned char crc_result = 0;

    for (int i = 0; i < CRC_BIT; ++i)
    {
        crc_result ^= data[i];
    }
    if (crc_result == data[CRC_BIT])
        return true;

    return false;
}

void StatusPublisher::Refresh()
{
    boost::mutex::scoped_lock lock(mMutex);
    static double theta_last = 0.0;
    static unsigned long time_stamp_last;
    static unsigned int ii = 0;
    static bool theta_updateflag = false;
    ii++;

    // std::cout << "Run in refresh:" << mbUpdated << std::endl;

    if (mbUpdated)
    {
        // Time
        ros::Time current_time = ros::Time::now();

        if (car_status.status == 0)
        {
            theta_updateflag = false;
        }
        else
        {
            theta_updateflag = true;
        }

        //pose computation.
        double car_velocity, delta_time, delta_car, delta_x, delta_y, delta_theta, var_len, var_angle;
        double angular_world_frame, Vx_world_frame, Vy_world_frame;

        //don't know how to use these.
        var_angle = (0.01f / 180.0f * PI) * (0.01f / 180.0f * PI);
        var_len = (50.0f / car_status.encoder_ppr * 2.0f * PI * wheel_radius) * (50.0f / car_status.encoder_ppr * 2.0f * PI * wheel_radius);

        //compute delta_t
        delta_time = car_status.time_stamp - time_stamp_last;
        time_stamp_last = car_status.time_stamp;

        //convert gyro (-180,180) to (0,360).
        if (car_status.gyro < 0)
            car_status.gyro += 360;

        angular_world_frame = car_status.gyro;

        //convert gyro frame to AGV frame. Use gyro's coordinate,not need to convert angular.
        if (angular_world_frame > 360)
        {
            angular_world_frame = angular_world_frame - 360;
        }

        //get delta_theta.
        delta_theta = angular_world_frame - theta_last;
        theta_last = angular_world_frame;
        std::cout << "angular_world_frame:" << angular_world_frame << std::endl;
        std::cout << "delta theta:" << delta_theta << std::endl;

        if (delta_theta > 5)
        {
        }

        delta_x = (car_status.velocity_x * cos(angular_world_frame * PI / 180) - car_status.velocity_y * sin(angular_world_frame * PI / 180)) * delta_time / 1000;
        delta_y = (car_status.velocity_x * sin(angular_world_frame * PI / 180) + car_status.velocity_y * cos(angular_world_frame * PI / 180)) * delta_time / 1000;

        //std::cout << "delta x:" << delta_x << std::endl;
        //std::cout << "delta y:" << delta_y << std::endl;
        //std::cout << "delta theta:"<< delta_theta << std::endl;

        if (delta_theta > 270)
            delta_theta -= 360;
        if (delta_theta < -270)
            delta_theta += 360;

        if (delta_theta > 20 || delta_theta < -20)
        {
            delta_theta = 0;
        }

        if (delta_x > 10 || delta_x < -10 || delta_y > 10 || delta_y < -10)
        {
            delta_x = 0;
            delta_y = 0;
        }

        //直接使用gyro的数据，并不通过累加的方法进行处理。
        CarPos2D.x += delta_x;
        CarPos2D.y += delta_y;
        CarPos2D.theta = angular_world_frame;

        //std::cout <<"angular from gyro:" << angular_world_frame;
        //std::cout << "Odom theta:" << CarPos2D.theta << std::endl;
        //std::cout << "Odom Sx:" << CarPos2D.x << std::endl;
        //std::cout << "Odom Sy:" << CarPos2D.y << std::endl;

        mPose2DPub.publish(CarPos2D);

        //flag
        std_msgs::Int32 flag;
        flag.data = car_status.status;

        //Twist
        double angle_speed;
        CarTwist.linear.x = car_status.velocity_x;
        CarTwist.linear.y = car_status.velocity_y;

        // the robot and the IMU are the same coord.
        CarTwist.angular.z = car_status.m_gyro_angular_vel * PI / 180;
        // CarTwist.angular.z = 0.0f;
        // std::cout << "angular z check:" << CarTwist.angular.z << std::endl;

        mTwistPub.publish(CarTwist);

        //not used for this version.
        // CarPower.data = car_status.power;
        // mPowerPub.publish(CarPower);

        CarOdom.header.stamp = current_time;
        CarOdom.header.frame_id = "odom";
        CarOdom.pose.pose.position.x = CarPos2D.x;
        CarOdom.pose.pose.position.y = CarPos2D.y;
        CarOdom.pose.pose.position.z = 0.0f;
        //std::cout << "angular in rad:" << CarPos2D.theta / 180.0f * PI << std::endl;

        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(CarPos2D.theta / 180.0f * PI);
        //std::cout << "odom_quat:" << odom_quat << std::endl;

        // geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0.0);
        CarOdom.pose.pose.orientation = odom_quat;
        CarOdom.pose.covariance = boost::assign::list_of(var_len)(0)(0)(0)(0)(0)(0)(var_len)(0)(0)(0)(0)(0)(0)(999)(0)(0)(0)(0)(0)(0)(999)(0)(0)(0)(0)(0)(0)(999)(0)(0)(0)(0)(0)(0)(var_angle);
        CarOdom.child_frame_id = "base_footprint";

        CarOdom.twist.twist.linear.x = CarTwist.linear.x;
        CarOdom.twist.twist.linear.y = CarTwist.linear.y;
        CarOdom.twist.twist.angular.z = CarTwist.angular.z;
        // std::cout << "angular vel:" << CarOdom.twist.twist.angular.z << std::endl;

        CarOdom.twist.covariance = boost::assign::list_of(var_len)(0)(0)(0)(0)(0)(0)(var_len)(0)(0)(0)(0)(0)(0)(999)(0)(0)(0)(0)(0)(0)(999)(0)(0)(0)(0)(0)(0)(999)(0)(0)(0)(0)(0)(0)(var_angle);
        mOdomPub.publish(CarOdom);

        //Use this to make sure all odom data are right.
        //std::cout << "Odom: Sx" << CarOdom.pose.pose.position.x << "Sy: " << CarOdom.pose.pose.position.y << "Vx: "
        // <<  CarOdom.twist.twist.linear.x << "Vy: " <<  CarOdom.twist.twist.linear.y << "Vz: " << CarOdom.twist.twist.angular.z << std::endl;

        // pub transform
        static tf::TransformBroadcaster br;
        tf::Quaternion q;
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(CarPos2D.x, CarPos2D.y, 0.0));
        q.setRPY(0, 0, CarPos2D.theta / 180 * PI);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_footprint"));

        ros::spinOnce();

        mbUpdated = false;
    }
}

double StatusPublisher::get_wheel_separation()
{
    return wheel_separation;
}

double StatusPublisher::get_wheel_radius()
{
    return wheel_radius;
}

int StatusPublisher::get_wheel_ppr()
{
    return car_status.encoder_ppr;
}

void StatusPublisher::get_wheel_speed(double speed[2])
{
    //右一左二
    speed[0] = car_status.omga_r / car_status.encoder_ppr * 2.0 * PI * wheel_radius;
    speed[1] = car_status.omga_l / car_status.encoder_ppr * 2.0 * PI * wheel_radius;
}

geometry_msgs::Pose2D StatusPublisher::get_CarPos2D()
{
    return CarPos2D;
}

geometry_msgs::Twist StatusPublisher::get_CarTwist()
{
    return CarTwist;
}

std_msgs::Float64 StatusPublisher::get_power()
{
    return CarPower;
}

nav_msgs::Odometry StatusPublisher::get_odom()
{
    return CarOdom;
}
int StatusPublisher::get_status()
{
    return car_status.status;
}

} //namespace xqserial_server
