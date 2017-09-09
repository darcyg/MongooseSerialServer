
#include "AsyncSerial.h"

#include <iostream>
#include <boost/thread.hpp>

#include <ros/console.h>
#include <ros/ros.h>
#include <ros/package.h>
#include "DiffDriverController.h"
#include "StatusPublisher.h"

using namespace std;

int main(int argc, char **argv)
{

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info)) { // Change the level to fit your needs
        ros::console::notifyLoggerLevelsChanged();
     }

    ROS_INFO("welcome to Mongoose serial server !");

    ros::init(argc, argv, "xqserial_server");
    ros::start();

    //获取串口参数
    std::string port;
    ros::param::param<std::string>("~port", port, "/dev/ttyUSB0");
    int baud;
    ros::param::param<int>("~baud", baud, 115200);
    cout << "port:" << port << " baud:" << baud << endl;

    //获取小车机械参数
    double separation = 0, radius = 0;
    bool DebugFlag = false;
    ros::param::param<double>("~wheel_separation", separation, 0.47);
    ros::param::param<double>("~wheel_radius", radius, 0.1016);
    ros::param::param<bool>("~debug_flag", DebugFlag, false);
    xqserial_server::StatusPublisher xq_status(separation, radius);

    //获取小车控制参数
    double max_speed;
    string cmd_topic;
    ros::param::param<double>("~max_speed", max_speed, 2.0);
    ros::param::param<std::string>("~cmd_topic", cmd_topic, "cmd_vel");

    try
    {
        CallbackAsyncSerial serial(port, baud);

        serial.setCallback(boost::bind(&xqserial_server::StatusPublisher::Update, &xq_status, _1, _2));
        xqserial_server::DiffDriverController xq_diffdriver(max_speed, cmd_topic, &xq_status, &serial);
        boost::thread cmd2serialThread(&xqserial_server::DiffDriverController::run, &xq_diffdriver);

        // send test flag
        char debugFlagCmd[] = {0xcd, 0xeb, 0xd7, 0x01, 'T'};
        if (DebugFlag)
        {
            std::cout << "Debug mode Enabled" << std::endl;
            serial.write(debugFlagCmd, 5);
        }

         char cmd_str1[14] = {0xFF, 0x03, 0x09, 0x00, 0x1E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xEB, 0xED};

         //发布周期为50hz
        ros::Rate r(50); 
        while (ros::ok())
        {
            if (serial.errorStatus() || serial.isOpen() == false)
            {
                cerr << "Error: serial port closed unexpectedly" << endl;
                break;
            }

            //定时发布状态小车状态
            xq_status.Refresh(); 
            r.sleep();
        }

    quit:
        serial.close();
    }
    catch (std::exception &e)
    {
        cerr << "Exception: " << e.what() << endl;
    }

    ros::shutdown();
    return 0;
}
