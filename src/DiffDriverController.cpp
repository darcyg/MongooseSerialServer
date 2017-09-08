#include "DiffDriverController.h"
#include <time.h>
#include <stdint.h>

namespace xqserial_server
{

DiffDriverController::DiffDriverController()
{
    max_wheelspeed = 2.0;
    cmd_topic = "cmd_vel";
    xq_status = new StatusPublisher();
    cmd_serial = NULL;
    MoveFlag = true;
}

DiffDriverController::DiffDriverController(double max_speed_, std::string cmd_topic_, StatusPublisher *xq_status_, CallbackAsyncSerial *cmd_serial_)
{
    MoveFlag = true;
    max_wheelspeed = max_speed_;
    cmd_topic = cmd_topic_;
    xq_status = xq_status_;
    cmd_serial = cmd_serial_;
}

void DiffDriverController::run()
{
    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe(cmd_topic, 1, &DiffDriverController::sendcmd, this);
    ros::Subscriber sub2 = nodeHandler.subscribe("/imu_cal", 1, &DiffDriverController::imuCalibration, this);
    ros::Subscriber sub3 = nodeHandler.subscribe("/globalMoveFlag", 1, &DiffDriverController::updateMoveFlag, this);
    ros::Subscriber sub4 = nodeHandler.subscribe("/barDetectFlag", 1, &DiffDriverController::updateBarDetectFlag, this);
    ros::spin();
}

void DiffDriverController::updateMoveFlag(const std_msgs::Bool &moveFlag)
{
    boost::mutex::scoped_lock lock(mMutex);
    MoveFlag = moveFlag.data;
}

void DiffDriverController::imuCalibration(const std_msgs::Bool &calFlag)
{
    if (calFlag.data)
    {
        //下发底层ｉｍｕ标定命令
        char cmd_str[5] = {0xcd, 0xeb, 0xd7, 0x01, 0x43};
        if (NULL != cmd_serial)
        {
            cmd_serial->write(cmd_str, 5);
        }
    }
}

void DiffDriverController::updateBarDetectFlag(const std_msgs::Bool &DetectFlag)
{
    if (DetectFlag.data)
    {
        //下发底层红外开启命令
        char cmd_str[6] = {0xcd, 0xeb, 0xd7, 0x02, 0x44, 0x01};
        if (NULL != cmd_serial)
        {
            cmd_serial->write(cmd_str, 6);
        }
    }
    else
    {
        //下发底层红外禁用命令
        char cmd_str[6] = {0xcd, 0xeb, 0xd7, 0x02, 0x44, 0x00};
        if (NULL != cmd_serial)
        {
            cmd_serial->write(cmd_str, 6);
        }
    }
}

void DiffDriverController::sendcmd(const geometry_msgs::Twist &command)
{
    static time_t t1 = time(NULL), t2;
    int i = 0, wheel_ppr = 1;
    int crc_length = 0;
    int crc = 0;
    bool stop_cmd = false;
    double separation = 0, radius = 0, speed_temp[2];
    int speed_lin_x = 0, speed_lin_y = 0, speed_ang = 0;
    char cmd_str[14] = {0xFF, 0x03, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF5, 0xED};
    char cmd_str1[14] = {0xFF, 0x03, 0x09, 0x00, 0x1E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xEB, 0xED};
    
    char stop_str[5] = {0xFF, 0x04, 0x00, 0xFB, 0xED};

    // if(xq_status->get_status()==0) return;//底层还在初始化

    separation = xq_status->get_wheel_separation();
    radius = xq_status->get_wheel_radius();
    wheel_ppr = xq_status->get_wheel_ppr();

    //坐标系转换，将move_base的坐标系转成小车的坐标系(World and AGV coordinates are the same)。
    speed_lin_x = command.linear.x * 100;
    speed_lin_y = command.linear.y * 100;
    speed_ang = command.angular.z * 100;
    // std::cout << "cmmond angular z:" << command.angular.z << std::endl;
    // std::cout << "speed_ang angular z:" << speed_ang << std::endl;

     //Get theta value.
    float speed[3] = {speed_lin_x, speed_lin_y, speed_ang};

    //put stop and velocity cmd.
    if (speed[0] == 0 && speed[1] == 0 && speed[2] == 0)
    {
        stop_cmd = true;
    }
    else
    {
        // std::cout << "This line:" << __LINE__ << std::endl;

        //put processed value to cmd_str.
        int Vx_state = 0, Vy_state = 0, Wz_state = 0;
        int Vx_tmp, Vy_tmp, Wz_tmp;
        uint32_t speed_vx, speed_vy, speed_wz;

        //vx
        if (speed[0] >= 0)
        {
            Vx_state = 0;
            speed_vx = speed[0];
        }
        else
        {
            Vx_state = 1;
            speed_vx = - speed[0];
        }
       
        Vx_tmp = ( speed_vx & 0x0000ff00) >> 8;
        cmd_str[3] = Vx_tmp;

        Vx_tmp = (speed_vx & 0x000000ff);
        cmd_str[4] = Vx_tmp;
        cmd_str[5] = Vx_state;

        //vy
        if (speed[1] >= 0)
        {
            Vy_state = 0;
            speed_vy = speed[1];
        }
        else
        {
            Vy_state = 1;
            speed_vy = -speed[1];
        }

        Vy_tmp = (speed_vy & 0x0000ff00) >> 8;
        cmd_str[6] = Vy_tmp;

        Vy_tmp = (speed_vy & 0x000000ff);
        cmd_str[7] = Vy_tmp;
        cmd_str[8] = Vy_state;

        //Wz
        if(speed[2] >= 0)
        {
            Wz_state = 0;
            speed_wz = speed[2];
        }
        else
        {
            Wz_state = 1;
            speed_wz = -speed[2];
        }

        Wz_tmp = (speed_wz & 0x0000ff00) >> 8;
        cmd_str[9] = Wz_tmp;

        Wz_tmp = (speed_wz & 0x000000ff);
        cmd_str[10] = Wz_tmp;
        cmd_str[11] = Wz_state;

        crc_length = (INFORMATION_LENGTH - 2);
        for (i = 0; i < crc_length; ++i)
        {
            crc = crc ^ cmd_str[i];
        }
        cmd_str[12] = (char)crc;

        cmd_str[13] = 0xED;
    }

    boost::mutex::scoped_lock lock(mMutex);
    if (!MoveFlag)
    {
        cmd_str[5] = 0x53;
        cmd_str[6] = 0x53;
    }
    if (NULL != cmd_serial)
    {
        if (stop_cmd)
        {
            cmd_serial->write(stop_str, 5);
        }
        else
        {
            cmd_serial->write(cmd_str, 14);
        }
    }
}

}
