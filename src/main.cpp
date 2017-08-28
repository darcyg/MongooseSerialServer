
#include "AsyncSerial.h"

#include <iostream>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <ros/package.h>
#include "DiffDriverController.h"
#include "StatusPublisher.h"

using namespace std;

int main(int argc, char **argv)
{
    ROS_INFO("welcome to xiaoqiang serial server,please feel free at home!");

    ros::init(argc, argv, "xqserial_server");
    ros::start();

    //获取串口参数
    std::string port;
    ros::param::param<std::string>("~port", port, "/dev/ttyUSB1");
    int baud;
    ros::param::param<int>("~baud", baud, 115200);
    cout<<"port:"<<port<<" baud:"<<baud<<endl;

    //获取小车机械参数
    double separation=0,radius=0;
    bool DebugFlag = false;
    ros::param::param<double>("~wheel_separation", separation, 0.37);
    ros::param::param<double>("~wheel_radius", radius, 0.15);
    ros::param::param<bool>("~debug_flag", DebugFlag, false);
    xqserial_server::StatusPublisher xq_status(separation,radius);

    //获取小车控制参数
    double max_speed;
    string cmd_topic;
    ros::param::param<double>("~max_speed", max_speed, 2.0);
    ros::param::param<std::string>("~cmd_topic", cmd_topic, "cmd_vel");

    try {
        CallbackAsyncSerial serial(port,baud);
        // ROS_INFO("This line:[%d]", __LINE__);

        serial.setCallback(boost::bind(&xqserial_server::StatusPublisher::Update,&xq_status,_1,_2));
        xqserial_server::DiffDriverController xq_diffdriver(max_speed,cmd_topic,&xq_status,&serial);
        boost::thread cmd2serialThread(& xqserial_server::DiffDriverController::run,&xq_diffdriver);

        // send test flag
        char debugFlagCmd[] = {0xcd,0xeb,0xd7,0x01, 'T'};
        if(DebugFlag){
            std::cout << "Debug mode Enabled" << std::endl;
            serial.write(debugFlagCmd, 5);
        }

        //Run a square,velocity is 0.2m/s. 20170811

#if 0


        char resetCmd1[] = {0xFF,0x02,0x04,0x14, 0x00, 0x9D, 0x00, 0x70, 0xED}; //速度0.2m/s,foward
        char resetCmd2[] = {0xFF,0x02,0x04,0x14, 0x02, 0x74, 0x00, 0x9B, 0xED}; //速度0.2m/s,right  6.28rad
        char resetCmd3[] = {0xFF,0x02,0x04,0x14, 0x01, 0xD7, 0x00, 0x3B, 0xED}; //速度0.2m/s,backward  4.71
        char resetCmd4[] = {0xFF,0x02,0x04,0x14, 0x01, 0x3A, 0x00, 0xD6, 0xED}; //速度0.2m/s,left   3.14
        char stopCmd[] = {0xFF, 0x04, 0x00, 0xFB, 0xED};

        serial.write(stopCmd, 5);
        sleep(0.5);

        sleep(3);
        // char resetCmd3[] = {0xFF,0x02,0x04,0x14, 0x0, 0x9D, 0x1E, 0x6E, 0xED}; //速度0.2m/s w = 0.3rad/s
        ROS_INFO("This line:[%d]", __LINE__);
        serial.write(resetCmd1, 9);
        sleep(12.5);

        serial.write(resetCmd2, 9);
        sleep(12.5);
        ROS_INFO("This line:[%d]", __LINE__);
        serial.write(resetCmd3, 9);
        sleep(12.5);

        serial.write(resetCmd4, 9);
        sleep(12.5);

        serial.write(stopCmd, 5);
        sleep(0.5);

        // std::cout << "Run in this line:" << __LINE__ << std::endl;

        // char resetCmd3[] = {0xFF,0x02,0x04,0x14, 0x0, 0x9D, 0x1E, 0x6E, 0xED}; //速度0.2m/s w = 0.3rad/s
        // serial.write(resetCmd3, 9);
        // sleep(12.5);
        //
        // serial.write(stopCmd, 5);
        // sleep(0.5);
        //
        // serial.write(resetCmd2, 9);
        // sleep(5);
        // serial.write(resetCmd3, 9);
        // sleep(5);
        //
        // serial.write(resetCmd2, 9);
        // sleep(5);
        // serial.write(resetCmd3, 9);
        // sleep(5);
        //
        //
        // serial.write(resetCmd2, 9);
        // sleep(5);
        // serial.write(resetCmd3, 9);
        // sleep(5);
#endif


#if 0 // Run a squarem velocity is 0.4m/s.
        //这段命令是0.4m/s
        char resetCmd5[] = {0xFF,0x02,0x04,0x28, 0x00, 0x9D, 0x00, 0x4C, 0xED}; //速度0.4m/s,foward
        char resetCmd6[] = {0xFF,0x02,0x04,0x28, 0x02, 0x74, 0x00, 0xA7, 0xED}; //速度0.4m/s,right  6.28rad
        char resetCmd7[] = {0xFF,0x02,0x04,0x28, 0x01, 0xD7, 0x00, 0x07, 0xED}; //速度0.4m/s,backward  4.71
        char resetCmd8[] = {0xFF,0x02,0x04,0x28, 0x01, 0x3A, 0x00, 0xEA, 0xED}; //速度0.4m/s,left   3.14


        sleep(3);
        // char resetCmd3[] = {0xFF,0x02,0x04,0x14, 0x0, 0x9D, 0x1E, 0x6E, 0xED}; //速度0.2m/s w = 0.3rad/s


        serial.write(resetCmd5, 9);
        sleep(6.25);

        serial.write(resetCmd6, 9);
        sleep(6.25);

        serial.write(resetCmd7, 9);
        sleep(6.28);

        serial.write(resetCmd8, 9);
        sleep(6.25);

        serial.write(stopCmd, 5);
        //sleep(0.5);

#endif
        //char test_resetCmd1[] = {0xFF,0x02,0x04,0x1E, 0x0, 0x9D, 0x1E, 0x64, 0xED}; //速度0.3m/s left
        //char test_resetCmd2[] = {0xFF,0x02,0x04,0x1E, 0x0, 0x9D, 0x0, 0x7A, 0xED}; //速度0.3m/s go straight

        //serial.write(test_resetCmd2, 9);
        //sleep(5.2);
        //        serial.write(test_resetCmd1, 9);
        //        sleep(5.2);
        //        serial.write(test_resetCmd2, 9);
        //        sleep(5.2);

        //        char stopCmd[] = {0xFF, 0x04, 0x00, 0xFB, 0xED};
        //        serial.write(stopCmd, 5);
        // char check_encoder[] = {0xFF, 0x06, 0x00, 0xF9, 0xED};
        // serial.write(check_encoder, 5);
        //std::cout << "Run in this line:" << __LINE__ << std::endl;
        //
        // sleep(3);
        //
        // // send fast stop cmd.
        // std::cout << "Run in this line:stop" << __LINE__ << std::endl;


        //serial.write(stopCmd, 5);

        ros::Rate r(50);//发布周期为50hz
        while (ros::ok())
        {
            if(serial.errorStatus() || serial.isOpen()==false)
            {
                cerr<<"Error: serial port closed unexpectedly"<<endl;
                break;
            }
            xq_status.Refresh();//定时发布状态
            r.sleep();
            //cout<<"run"<<endl;
        }

quit:
        serial.close();

    } catch (std::exception& e) {
        cerr<<"Exception: "<<e.what()<<endl;
    }

    ros::shutdown();
    return 0;
}
