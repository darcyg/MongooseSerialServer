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
    // ROS_INFO("This line:[%d]", __LINE__);

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

// void DiffDriverController::sendcmd(const geometry_msgs::Twist &command)
// {
//     static time_t t1=time(NULL),t2;
//     int i=0,wheel_ppr=1;
//     double separation=0,radius=0,speed_lin=0,speed_ang=0,speed_temp[2];
//     char speed[2]={0,0};//右一左二
//     char cmd_str[13]={0xcd,0xeb,0xd7,0x09,0x74,0x53,0x53,0x53,0x53,0x00,0x00,0x00,0x00};

//     ROS_INFO("Run in this line:[%d]", __LINE__);

//     if(xq_status->get_status()==0) return;//底层还在初始化
//     separation=xq_status->get_wheel_separation();
//     radius=xq_status->get_wheel_radius();
//     wheel_ppr=xq_status->get_wheel_ppr();

//     //转换速度单位，由米转换成转
//     speed_lin=command.linear.x/(2.0*PI*radius);
//     speed_ang=command.angular.z*separation/(2.0*PI*radius);

//     float scale=std::max(std::abs(speed_lin+speed_ang/2.0),std::abs(speed_lin-speed_ang/2.0))/max_wheelspeed;
//     if(scale>1.0)
//     {
//       scale=1.0/scale;
//     }
//     else
//     {
//       scale=1.0;
//     }
//     //转出最大速度百分比,并进行限幅
//     speed_temp[0]=scale*(speed_lin+speed_ang/2)/max_wheelspeed*100.0;
//     speed_temp[0]=std::min(speed_temp[0],100.0);
//     speed_temp[0]=std::max(-100.0,speed_temp[0]);

//     speed_temp[1]=scale*(speed_lin-speed_ang/2)/max_wheelspeed*100.0;
//     speed_temp[1]=std::min(speed_temp[1],100.0);
//     speed_temp[1]=std::max(-100.0,speed_temp[1]);

//   //std::cout<<"radius "<<radius<<std::endl;
//   //std::cout<<"ppr "<<wheel_ppr<<std::endl;
//   //std::cout<<"pwm "<<speed_temp[0]<<std::endl;
//   //  command.linear.x/
//     for(i=0;i<2;i++)
//     {
//      speed[i]=speed_temp[i];
//      if(speed[i]<0)
//      {
//          cmd_str[5+i]=0x42;//B
//          cmd_str[9+i]=-speed[i];
//      }
//      else if(speed[i]>0)
//      {
//          cmd_str[5+i]=0x46;//F
//          cmd_str[9+i]=speed[i];
//      }
//      else
//      {
//          cmd_str[5+i]=0x53;//S
//          cmd_str[9+i]=0x00;
//      }
//     }

//     // std::cout<<"distance1 "<<xq_status->car_status.distance1<<std::endl;
//     // std::cout<<"distance2 "<<xq_status->car_status.distance2<<std::endl;
//     // std::cout<<"distance3 "<<xq_status->car_status.distance3<<std::endl;
//     // if(xq_status->get_status()==2)
//     // {
//     //   //有障碍物
//     //   if(xq_status->car_status.distance1<30&&xq_status->car_status.distance1>0&&cmd_str[6]==0x46)
//     //   {
//     //     cmd_str[6]=0x53;
//     //   }
//     //   if(xq_status->car_status.distance2<30&&xq_status->car_status.distance2>0&&cmd_str[5]==0x46)
//     //   {
//     //     cmd_str[5]=0x53;
//     //   }
//     //   if(xq_status->car_status.distance3<20&&xq_status->car_status.distance3>0&&(cmd_str[5]==0x42||cmd_str[6]==0x42))
//     //   {
//     //     cmd_str[5]=0x53;
//     //     cmd_str[6]=0x53;
//     //   }
//     //   if(xq_status->car_status.distance1<15&&xq_status->car_status.distance1>0&&(cmd_str[5]==0x46||cmd_str[6]==0x46))
//     //   {
//     //     cmd_str[5]=0x53;
//     //     cmd_str[6]=0x53;
//     //   }
//     //   if(xq_status->car_status.distance2<15&&xq_status->car_status.distance2>0&&(cmd_str[5]==0x46||cmd_str[6]==0x46))
//     //   {
//     //     cmd_str[5]=0x53;
//     //     cmd_str[6]=0x53;
//     //   }
//     // }

//     boost::mutex::scoped_lock lock(mMutex);
//     if(!MoveFlag)
//     {
//       cmd_str[5]=0x53;
//       cmd_str[6]=0x53;
//     }
//     if(NULL!=cmd_serial)
//     {
//         cmd_serial->write(cmd_str,13);
//     }

//    // command.linear.x
// }

void DiffDriverController::sendcmd(const geometry_msgs::Twist &command)
{
    static time_t t1 = time(NULL), t2;
    int i = 0, wheel_ppr = 1;
    int crc_length = 0;
    int crc = 0;
    bool stop_cmd = false;
    double separation = 0, radius = 0, speed_lin_x = 0, speed_lin_y = 0, speed_temp[2];
    double speed_ang = 0;
    // char speed[2]={0,0};//右一左二
    // char speed[1] = {0};
    char cmd_str[9] = {0xFF, 0x02, 0x04, 0x14, 0x00, 0x9D, 0x00, 0x70, 0xED};
    char stop_str[5] = {0xFF, 0x04, 0x00, 0xFB, 0xED};
    // char resetCmd4[9] = {0xFF, 0x02, 0x04, 0x14, 0x00, 0x9D, 0x00, 0x70, 0xED}; //速度0.2m/s,forward   3.14

    // ROS_INFO("Run in this line xqserver:[%d]", __LINE__);
    // std::cout << "This line:" << __LINE__ << std::endl;

    // if(xq_status->get_status()==0) return;//底层还在初始化

    separation = xq_status->get_wheel_separation();
    radius = xq_status->get_wheel_radius();
    wheel_ppr = xq_status->get_wheel_ppr();

    //转换速度单位，由米转换成转,not need for this code.
    // speed_lin_x = command.linear.x / (2.0 * PI * radius);
    // speed_lin_y = command.linear.y / (2.0 * PI * radius);
    // speed_ang = command.angular.z * separation / (2.0 * PI * radius);

    //坐标系转换，将move_base的坐标系转成小车的坐标系。
    speed_lin_y = command.linear.x;
    speed_lin_x = -command.linear.y;
    speed_ang = command.angular.z;

    //Get theta value.
    double speed[3] = {speed_lin_x, speed_lin_y, speed_ang};
    uint32_t anglar;

    // std::cout << "x velocity:" << command.linear.x << std::endl;
    // std::cout << "y velocity:" << command.linear.y << std::endl;
    // std::cout << "angular velocity:" << command.angular.z << std::endl;
    // ROS_INFO("linear velocity:[%d]", (float)command.linear.x);

    double speedValue = sqrt(speed[0] * speed[0] + speed[1] * speed[1]) * 100;
    // ROS_INFO("speed:[%d]", speedValue);

    //确定数据在哪个相限。
    if (speed[0] == 0 || speed[1] == 0)
    {
        if (speed[0] == 0)
        {
            if (speed[1] > 0)
                anglar = 157;
            else
                anglar = 471;
        }

        if (speed[1] == 0)
        {
            if (speed[0] >= 0)
                anglar = 0;
            else
                anglar = 628;
        }
    }
    else
    {
        if (speed[0] > 0 && speed[1] > 0)
        {
            anglar = atan(speed[1] / speed[0]) * 100;
        }
        else if (speed[0] > 0 && speed[1] < 0)
        {
            anglar = atan(speed[1] / speed[0]) * 100 + 628;
        }
        else if (speed[0] < 0 && speed[1] > 0)
        {
            anglar = atan(speed[1] / speed[0]) * 100 + 314;
        }
        else if (speed[0] < 0 && speed[1] < 0)
        {
            anglar = atan(speed[1] / speed[0]) * 100 + 314;
        }
    }

    //put stop and velocity cmd.
    if (speedValue == 0 && anglar == 0)
    {
        stop_cmd = true;
    }
    else
    {
        //put processed value to cmd_str.
        cmd_str[3] = speedValue;

        int temp_byte = (anglar & 0x0000ff00) >> 8;

        cmd_str[4] = temp_byte;
        temp_byte = (anglar & 0x000000ff);

        cmd_str[5] = temp_byte;

        cmd_str[6] = (speed_ang * 100);

        crc_length = (INFORMATION_LENGTH - 2);
        for (i = 0; i < crc_length; ++i)
        {
            crc = crc ^ cmd_str[i];
        }
        cmd_str[7] = (char)crc;

        cmd_str[8] = 0xED;
    }

    // for(int i = 0; i < 9; ++i)
    // {
    //     std::cout << "cmd_str value:" << i << ":" << cmd_str[i] << std::endl;
    // }

    // std::cout<<"distance1 "<<xq_status->car_status.distance1<<std::endl;
    // std::cout<<"distance2 "<<xq_status->car_status.distance2<<std::endl;
    // std::cout<<"distance3 "<<xq_status->car_status.distance3<<std::endl;
    // if(xq_status->get_status()==2)
    // {
    //   //有障碍物
    //   if(xq_status->car_status.distance1<30&&xq_status->car_status.distance1>0&&cmd_str[6]==0x46)
    //   {
    //     cmd_str[6]=0x53;
    //   }
    //   if(xq_status->car_status.distance2<30&&xq_status->car_status.distance2>0&&cmd_str[5]==0x46)
    //   {
    //     cmd_str[5]=0x53;
    //   }
    //   if(xq_status->car_status.distance3<20&&xq_status->car_status.distance3>0&&(cmd_str[5]==0x42||cmd_str[6]==0x42))
    //   {
    //     cmd_str[5]=0x53;
    //     cmd_str[6]=0x53;
    //   }
    //   if(xq_status->car_status.distance1<15&&xq_status->car_status.distance1>0&&(cmd_str[5]==0x46||cmd_str[6]==0x46))
    //   {
    //     cmd_str[5]=0x53;
    //     cmd_str[6]=0x53;
    //   }
    //   if(xq_status->car_status.distance2<15&&xq_status->car_status.distance2>0&&(cmd_str[5]==0x46||cmd_str[6]==0x46))
    //   {
    //     cmd_str[5]=0x53;
    //     cmd_str[6]=0x53;
    //   }
    // }

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
            cmd_serial->write(cmd_str, 9);
        }
    }
}

void DiffDriverController::sendcmdTest(const geometry_msgs::Twist &command)
{
    static time_t t1 = time(NULL), t2;
    int i = 0, wheel_ppr = 1;
    double separation = 0, radius = 0, speed_lin = 0, speed_ang = 0, speed_temp[2];
    char speed[2] = {0, 0}; //右一左二
    char cmd_str[13] = {0xcd, 0xeb, 0xd7, 0x09, 0x74, 0x53, 0x53, 0x53, 0x53, 0x00, 0x00, 0x00, 0x00};

    ROS_INFO("Run in this line:[%d]", __LINE__);

    if (xq_status->get_status() == 0)
        return; //底层还在初始化
    separation = xq_status->get_wheel_separation();
    radius = xq_status->get_wheel_radius();
    wheel_ppr = xq_status->get_wheel_ppr();

    //转换速度单位，由米转换成转
    speed_lin = command.linear.x / (2.0 * PI * radius);
    speed_ang = command.angular.z * separation / (2.0 * PI * radius);

    float scale = std::max(std::abs(speed_lin + speed_ang / 2.0), std::abs(speed_lin - speed_ang / 2.0)) / max_wheelspeed;
    if (scale > 1.0)
    {
        scale = 1.0 / scale;
    }
    else
    {
        scale = 1.0;
    }
    //转出最大速度百分比,并进行限幅
    speed_temp[0] = scale * (speed_lin + speed_ang / 2) / max_wheelspeed * 100.0;
    speed_temp[0] = std::min(speed_temp[0], 100.0);
    speed_temp[0] = std::max(-100.0, speed_temp[0]);

    speed_temp[1] = scale * (speed_lin - speed_ang / 2) / max_wheelspeed * 100.0;
    speed_temp[1] = std::min(speed_temp[1], 100.0);
    speed_temp[1] = std::max(-100.0, speed_temp[1]);

    //std::cout<<"radius "<<radius<<std::endl;
    //std::cout<<"ppr "<<wheel_ppr<<std::endl;
    //std::cout<<"pwm "<<speed_temp[0]<<std::endl;
    //  command.linear.x/
    for (i = 0; i < 2; i++)
    {
        speed[i] = speed_temp[i];
        if (speed[i] < 0)
        {
            cmd_str[5 + i] = 0x42; //B
            cmd_str[9 + i] = -speed[i];
        }
        else if (speed[i] > 0)
        {
            cmd_str[5 + i] = 0x46; //F
            cmd_str[9 + i] = speed[i];
        }
        else
        {
            cmd_str[5 + i] = 0x53; //S
            cmd_str[9 + i] = 0x00;
        }
    }

    // std::cout<<"distance1 "<<xq_status->car_status.distance1<<std::endl;
    // std::cout<<"distance2 "<<xq_status->car_status.distance2<<std::endl;
    // std::cout<<"distance3 "<<xq_status->car_status.distance3<<std::endl;
    // if(xq_status->get_status()==2)
    // {
    //   //有障碍物
    //   if(xq_status->car_status.distance1<30&&xq_status->car_status.distance1>0&&cmd_str[6]==0x46)
    //   {
    //     cmd_str[6]=0x53;
    //   }
    //   if(xq_status->car_status.distance2<30&&xq_status->car_status.distance2>0&&cmd_str[5]==0x46)
    //   {
    //     cmd_str[5]=0x53;
    //   }
    //   if(xq_status->car_status.distance3<20&&xq_status->car_status.distance3>0&&(cmd_str[5]==0x42||cmd_str[6]==0x42))
    //   {
    //     cmd_str[5]=0x53;
    //     cmd_str[6]=0x53;
    //   }
    //   if(xq_status->car_status.distance1<15&&xq_status->car_status.distance1>0&&(cmd_str[5]==0x46||cmd_str[6]==0x46))
    //   {
    //     cmd_str[5]=0x53;
    //     cmd_str[6]=0x53;
    //   }
    //   if(xq_status->car_status.distance2<15&&xq_status->car_status.distance2>0&&(cmd_str[5]==0x46||cmd_str[6]==0x46))
    //   {
    //     cmd_str[5]=0x53;
    //     cmd_str[6]=0x53;
    //   }
    // }

    boost::mutex::scoped_lock lock(mMutex);
    if (!MoveFlag)
    {
        cmd_str[5] = 0x53;
        cmd_str[6] = 0x53;
    }
    if (NULL != cmd_serial)
    {
        cmd_serial->write(cmd_str, 13);
    }

    // command.linear.x
}
}
