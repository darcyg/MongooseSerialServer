#include "StatusPublisher.h"
#include "AsyncSerial.h"
#include <memory.h>
#include <math.h>
#include <stdlib.h>

#define DISABLE 0
#define ENABLE 1

//serial package data.
#define package_begin 255
#define commond_bit 249
#define package_end 237
#define package_len 17
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
    mbUpdated=false;
    wheel_separation=0.37;
    wheel_radius=0.06;

    CarPos2D.x=0.0;
    CarPos2D.y=0.0;
    CarPos2D.theta=0.0;

    CarTwist.linear.x=0.0;
    CarTwist.linear.y=0.0;
    CarTwist.linear.z=0.0;
    CarTwist.angular.x=0.0;
    CarTwist.angular.y=0.0;
    CarTwist.angular.z=0.0;

    CarPower.data = 0.0;

    int i=0;
    int * status;
    status=(int *)&car_status;
    for(i=0;i<23;i++)
    {
        status[i]=0;
    }
    car_status.encoder_ppr=4*12*64;

    mPose2DPub = mNH.advertise<geometry_msgs::Pose2D>("xqserial_server/Pose2D",1,true);
    mStatusFlagPub = mNH.advertise<std_msgs::Int32>("xqserial_server/StatusFlag",1,true);
    mTwistPub = mNH.advertise<geometry_msgs::Twist>("xqserial_server/Twist",1,true);
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

StatusPublisher::StatusPublisher(double separation,double radius)
{
    new (this)StatusPublisher();
    wheel_separation=separation;
    wheel_radius=radius;
}

void StatusPublisher::Update(const char data[], unsigned int len)
{
    // if(len <1) return;
    // static char data2[1024];
    // static int len2=0;
    boost::mutex::scoped_lock lock(mMutex);

    int i=0,j=0;
    int * receive_byte;
    static unsigned int last_data[2]={0x00,0x00};
    static unsigned char last_str[2]={0x00,0x00};
    static unsigned char new_packed_ctr=DISABLE;//ENABLE表示新包开始，DISABLE 表示上一个包还未处理完；
    static int new_packed_ok_len=0;//包的理论长度
    static int new_packed_len=0;//包的实际长度
    static unsigned char cmd_string_buf[17];
    unsigned char current_str=0x00;
    const int cmd_string_max_size=512;
    receive_byte=(int *)&car_status;
    //int ii=0;
    //boost::mutex::scoped_lock lock(mMutex);

     if(len<119)
     {
     std::cout<<"len0:"<<len<<std::endl;
       current_str=data[0];
       std::cout<<(unsigned int)current_str<<std::endl;
     }

    for(i=0;i<len;i++)
    {
        current_str=data[i];
        // unsigned int temp=(unsigned int)current_str;
        // std::cout<<temp<<std::endl;
        //判断是否有新包头
        if(last_str[0] == package_end && last_str[1] == package_begin && current_str == commond_bit) //包头 205 235 215
        {
            //            std::cout<<"runup1 "<<std::endl;
            new_packed_ctr=ENABLE;
            new_packed_ok_len=0;
            new_packed_len=new_packed_ok_len;
            last_str[0]=last_str[1];//保存最后两个字符，用来确定包头
            last_str[1]=current_str;
        }
        last_str[0]=last_str[1];//保存最后两个字符，用来确定包头
        last_str[1]=current_str;

        if (new_packed_ctr==ENABLE)
        {

            //获取包长度
            new_packed_ok_len=current_str;
            if(new_packed_ok_len>cmd_string_max_size)
                new_packed_ok_len=cmd_string_max_size; //包内容最大长度有限制

            //            std::cout<<"runup2 "<< new_packed_len<< new_packed_ok_len<<std::endl;
        }
        if(last_data[0] = package_end && current_str == package_begin)
        {
            //            std::cout << "start to parse data: " << std::endl;
            start_parse = true;
            receive_data_size = 0;

        }
        //        last_data[0] = last_data[1];
        last_data[0] = current_str;

        if(start_parse)
        {
            vdataBuffer.push_back(current_str);
            if(vdataBuffer.size() == package_len )
            {
                STM32_Encoder_IMU_Parse(vdataBuffer);

                mbUpdated = true;
                start_parse = false;
//                for(int j = 0; j < 17; ++j)
//                {
//                     std::cout << "data:" << (int)vdataBuffer[j] << std::endl;
//                }
                vdataBuffer.clear();
            }
        }

    }

    return;
}


void StatusPublisher::STM32_Encoder_IMU_Parse(std::vector<unsigned char> data)
{
    unsigned char i=0;
    unsigned char command = 0;
    unsigned long FR_spin_speed_temp = 0;
    unsigned long FL_spin_speed_temp = 0;
    unsigned long BL_spin_speed_temp = 0;
    unsigned long BR_spin_speed_temp = 0;

    float 	FR_spin_speed = 0.0;
    float 	FL_spin_speed = 0.0;
    float 	BL_spin_speed = 0.0;
    float 	BR_spin_speed = 0.0;

    bool crc_ok = false;
    crc_ok = crc_check(data);

    if(crc_ok)
    {
        command = data[1];

        switch(command)
        {
        case 0x01:
            break;

        case 0xF9:
            FR_spin_speed_temp |= data[3];
            FR_spin_speed_temp = (FR_spin_speed_temp << 16);
            FR_spin_speed_temp |= data[4];
            FR_spin_speed_temp = (FR_spin_speed_temp << 8);
            FR_spin_speed_temp |= data[5];
            FR_spin_speed = (float)(FR_spin_speed_temp / 100.0);

            //Convert n/min to m/s.
            car_status.omga_FR = FR_spin_speed * 2 * Pi * wheel_radius / 60;
            std::cout << "FR spin speed:" << car_status.omga_FR << std::endl;

            FL_spin_speed_temp |= data[6];
            FL_spin_speed_temp = (FL_spin_speed_temp << 16);
            FL_spin_speed_temp |= data[7];
            FL_spin_speed_temp = (FL_spin_speed_temp << 8);
            FL_spin_speed_temp |= data[8];
            FL_spin_speed = (float)(FL_spin_speed_temp / 100.0);
            car_status.omga_FL = FL_spin_speed * 2 * Pi * wheel_radius / 60;
//            std::cout << "FL spin speed:" << FL_spin_speed << std::endl;

            //this encoder data error.
            BL_spin_speed_temp |= data[9];
            BL_spin_speed_temp = (BL_spin_speed_temp << 16);
            BL_spin_speed_temp |= data[10];
            BL_spin_speed_temp = (BL_spin_speed_temp << 8);
            BL_spin_speed_temp |= data[11];
            BL_spin_speed = (float)(BL_spin_speed_temp / 100.0);
            car_status.omga_BL = BL_spin_speed * 2 * Pi * wheel_radius / 60;
//            std::cout << "BL spin speed:" << BL_spin_speed << std::endl;

            BR_spin_speed_temp |= data[12];
            BR_spin_speed_temp = (BR_spin_speed_temp << 16);
            BR_spin_speed_temp |= data[13];
            BR_spin_speed_temp = (BR_spin_speed_temp << 8);
            BR_spin_speed_temp |= data[14];
            BR_spin_speed = (float)(BR_spin_speed_temp / 100.0);
            car_status.omga_BR = BR_spin_speed * 2 * Pi * wheel_radius /60;
//            std::cout << "BR spin speed:" << BR_spin_speed << std::endl;

            break;

        default: break;
        }
    }
}	

bool StatusPublisher::crc_check(std::vector<unsigned char> data)
{
    unsigned char  crc_result = 0;

    for(int i = 0; i < 15; ++i)
    {
        crc_result ^= data[i];
//        std::cout << "crc:" << (int)crc_result << std::endl;
//        std::cout << "data:" << (int)data[i] << std::endl;
    }
    if(crc_result == data[15])
        return true;

    return false;
}

void StatusPublisher::Refresh()
{
    boost::mutex::scoped_lock lock(mMutex);
    static double theta_last=0.0;
    static unsigned int ii=0;
    static bool theta_updateflag = false;
    ii++;
//    std::cout<<"runR"<< mbUpdated<<std::endl;
    if(mbUpdated)
    {
        // Time
        ros::Time current_time = ros::Time::now();

        if(car_status.status == 0)
        {
            theta_updateflag = false;
        }
        else
        {
            theta_updateflag = true;
        }
        //pose

        double delta_car,delta_x,delta_y,delta_theta,var_len,var_angle;


        var_len=(50.0f/car_status.encoder_ppr*2.0f*PI*wheel_radius)*(50.0f/car_status.encoder_ppr*2.0f*PI*wheel_radius);
        var_angle=(0.01f/180.0f*PI)*(0.01f/180.0f*PI);

        delta_car=(car_status.encoder_delta_r+car_status.encoder_delta_l)/2.0f*1.0f/car_status.encoder_ppr*2.0f*PI*wheel_radius;
        if(delta_car>0.05||delta_car<-0.05)
        {
            // std::cout<<"get you!"<<std::endl;
            delta_car = 0;
        }
        // if(ii%50==0||car_status.encoder_delta_car>3000||car_status.encoder_delta_car<-3000)
        // {
        //   std::cout<<"delta_encoder_car:"<< car_status.encoder_delta_car <<std::endl;
        //   std::cout<<"delta_encoder_r:"<< car_status.encoder_delta_r <<std::endl;
        //   std::cout<<"delta_encoder_l:"<< car_status.encoder_delta_l <<std::endl;
        //   std::cout<<"ppr:"<< car_status.encoder_ppr <<std::endl;
        //   std::cout<<"delta_car:"<< delta_car <<std::endl;
        // }

        delta_x=delta_car*cos(CarPos2D.theta* PI / 180.0f);
        delta_y=delta_car*sin(CarPos2D.theta* PI / 180.0f);

        delta_theta=car_status.theta-theta_last;
        theta_last=car_status.theta;
        if(delta_theta > 270 ) delta_theta -= 360;
        if(delta_theta < -270 ) delta_theta += 360;

        if((!theta_updateflag) ||delta_theta>20||delta_theta<-20)
        {
            delta_theta = 0;
        }

        CarPos2D.x+=delta_x;
        CarPos2D.y+=delta_y;
        CarPos2D.theta+=delta_theta;

        if(CarPos2D.theta>360.0) CarPos2D.theta-=360;
        if(CarPos2D.theta<0.0) CarPos2D.theta+=360;

        mPose2DPub.publish(CarPos2D);

        //flag
        std_msgs::Int32 flag;
        flag.data=car_status.status;

        //底层障碍物信息
        if((car_status.distance1+car_status.distance2+car_status.distance3+car_status.distance4)>0.1&&(car_status.distance1+car_status.distance2+car_status.distance3+car_status.distance4)<5.0)
        {
            //有障碍物
            flag.data=2;
        }
        mStatusFlagPub.publish(flag);

        int barArea_nums=0;
        int clearArea_nums=0;
        if(car_status.distance1>0.1)
        {
            barArea_nums+=3;
        }else{
            clearArea_nums+=6;
        }
        if(car_status.distance2>0.1)
        {
            barArea_nums+=3;
        }else{
            clearArea_nums+=6;
        }
        if(car_status.distance4>0.1)
        {
            barArea_nums+=3;
        }else{
            clearArea_nums+=6;
        }

        if(barArea_nums>0)
        {
            //发布雷区
            PointCloud::Ptr barcloud_msg(new PointCloud);
            barcloud_msg->header.stamp = current_time;
            barcloud_msg->height = 1;
            barcloud_msg->width  = barArea_nums;
            barcloud_msg->is_dense = true;
            barcloud_msg->is_bigendian = false;
            barcloud_msg->header.frame_id="kinect_link_new";
            sensor_msgs::PointCloud2Modifier pcd_modifier1(*barcloud_msg);
            pcd_modifier1.setPointCloud2FieldsByString(1,"xyz");
            sensor_msgs::PointCloud2Iterator<float> bariter_x(*barcloud_msg, "x");
            sensor_msgs::PointCloud2Iterator<float> bariter_y(*barcloud_msg, "y");
            sensor_msgs::PointCloud2Iterator<float> bariter_z(*barcloud_msg, "z");
            if(car_status.distance2>0.1)
            {
                for(int k=0;k<3;k++,++bariter_x, ++bariter_y,++bariter_z)
                {
                    *bariter_x=0.3;
                    *bariter_y=-0.10-k*0.05;
                    *bariter_z=0.15;
                }
            }
            if(car_status.distance4>0.1)
            {
                for(int k=0;k<3;k++,++bariter_x, ++bariter_y,++bariter_z)
                {
                    *bariter_x=0.3;
                    *bariter_y=-0.1+k*0.05;
                    *bariter_z=0.15;
                }
            }
            if(car_status.distance1>0.1)
            {
                for(int k=0;k<3;k++,++bariter_x, ++bariter_y,++bariter_z)
                {
                    *bariter_x=0.3;
                    *bariter_y=0.05+k*0.05;
                    *bariter_z=0.15;
                }
            }
            if(ii%5==0)
            {
                pub_barpoint_cloud_.publish(barcloud_msg);
            }
        }
        if(clearArea_nums>0)
        {
            //发布雷区
            PointCloud::Ptr clearcloud_msg(new PointCloud);
            clearcloud_msg->header.stamp = current_time;
            clearcloud_msg->height = 1;
            clearcloud_msg->width  = clearArea_nums;
            clearcloud_msg->is_dense = true;
            clearcloud_msg->is_bigendian = false;
            clearcloud_msg->header.frame_id="kinect_link_new";
            sensor_msgs::PointCloud2Modifier pcd_modifier1(*clearcloud_msg);
            pcd_modifier1.setPointCloud2FieldsByString(1,"xyz");
            sensor_msgs::PointCloud2Iterator<float> cleariter_x(*clearcloud_msg, "x");
            sensor_msgs::PointCloud2Iterator<float> cleariter_y(*clearcloud_msg, "y");
            sensor_msgs::PointCloud2Iterator<float> cleariter_z(*clearcloud_msg, "z");
            if(car_status.distance2<0.1)
            {
                for(int k=0;k<3;k++,++cleariter_x, ++cleariter_y,++cleariter_z)
                {
                    *cleariter_x=0.3;
                    *cleariter_y=-0.1-k*0.05;
                    *cleariter_z=0.0;
                }
                for(int k=0;k<3;k++,++cleariter_x, ++cleariter_y,++cleariter_z)
                {
                    *cleariter_x=0.25;
                    *cleariter_y=-0.1-k*0.05;
                    *cleariter_z=0.0;
                }
            }
            if(car_status.distance4<0.1)
            {
                for(int k=0;k<3;k++,++cleariter_x, ++cleariter_y,++cleariter_z)
                {
                    *cleariter_x=0.3;
                    *cleariter_y=-0.1+k*0.05;
                    *cleariter_z=0.0;
                }
                for(int k=0;k<3;k++,++cleariter_x, ++cleariter_y,++cleariter_z)
                {
                    *cleariter_x=0.25;
                    *cleariter_y=-0.1+k*0.05;
                    *cleariter_z=0.0;
                }
            }
            if(car_status.distance1<0.1)
            {
                for(int k=0;k<3;k++,++cleariter_x, ++cleariter_y,++cleariter_z)
                {
                    *cleariter_x=0.3;
                    *cleariter_y=0.05+k*0.05;
                    *cleariter_z=0.0;
                }
                for(int k=0;k<3;k++,++cleariter_x, ++cleariter_y,++cleariter_z)
                {
                    *cleariter_x=0.25;
                    *cleariter_y=0.05+k*0.05;
                    *cleariter_z=0.0;
                }
            }
            if(ii%5==0)
            {
                pub_clearpoint_cloud_.publish(clearcloud_msg);
            }
        }

        //Twist
        double angle_speed;
        CarTwist.linear.x=delta_car*50.0f;
        angle_speed=-car_status.IMU[5];
        CarTwist.angular.z=angle_speed * PI /180.0f;
        mTwistPub.publish(CarTwist);

        CarPower.data = car_status.power;
        mPowerPub.publish(CarPower);

        CarOdom.header.stamp = current_time;
        CarOdom.header.frame_id = "odom";
        CarOdom.pose.pose.position.x = CarPos2D.x;
        CarOdom.pose.pose.position.y = CarPos2D.y;
        CarOdom.pose.pose.position.z = 0.0f;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(CarPos2D.theta/180.0f*PI);
        CarOdom.pose.pose.orientation = odom_quat;
        CarOdom.pose.covariance =  boost::assign::list_of(var_len) (0) (0)  (0)  (0)  (0)
                (0) (var_len)  (0)  (0)  (0)  (0)
                (0)   (0)  (999) (0)  (0)  (0)
                (0)   (0)   (0) (999) (0)  (0)
                (0)   (0)   (0)  (0) (999) (0)
                (0)   (0)   (0)  (0)  (0)  (var_angle) ;
        CarOdom.child_frame_id = "base_footprint";
//        CarOdom.twist.twist.linear.x = CarTwist.linear.x;// * cos(CarPos2D.theta* PI / 180.0f);
        CarOdom.twist.twist.linear.x = 0.2;// * cos(CarPos2D.theta* PI / 180.0f);
        CarOdom.twist.twist.linear.y = CarTwist.linear.y;// * sin(CarPos2D.theta* PI / 180.0f);
        CarOdom.twist.twist.angular.z = CarTwist.angular.z;
        CarOdom.twist.covariance =  boost::assign::list_of(var_len) (0) (0)  (0)  (0)  (0)
                (0) (var_len)  (0)  (0)  (0)  (0)
                (0)   (0)  (999) (0)  (0)  (0)
                (0)   (0)   (0) (999) (0)  (0)
                (0)   (0)   (0)  (0) (999) (0)
                (0)   (0)   (0)  (0)  (0)  (var_angle) ;
        mOdomPub.publish(CarOdom);

        // pub transform

        static tf::TransformBroadcaster br;
        tf::Quaternion q;
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(CarPos2D.x, CarPos2D.y, 0.0) );
        q.setRPY(0, 0, CarPos2D.theta/180*PI);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_footprint"));

        ros::spinOnce();

        mbUpdated = false;
    }
}

double StatusPublisher::get_wheel_separation(){
    return wheel_separation;
}

double StatusPublisher::get_wheel_radius(){
    return wheel_radius;
}

int StatusPublisher::get_wheel_ppr(){
    return car_status.encoder_ppr;
}

void StatusPublisher::get_wheel_speed(double speed[2]){
    //右一左二
    speed[0]=car_status.omga_r/car_status.encoder_ppr*2.0*PI*wheel_radius;
    speed[1]=car_status.omga_l/car_status.encoder_ppr*2.0*PI*wheel_radius;
}

geometry_msgs::Pose2D StatusPublisher::get_CarPos2D(){
    return CarPos2D;
}

geometry_msgs::Twist StatusPublisher::get_CarTwist(){
    return CarTwist;
}

std_msgs::Float64 StatusPublisher::get_power(){
    return CarPower;
}

nav_msgs::Odometry StatusPublisher::get_odom(){
    return CarOdom;
}
int StatusPublisher::get_status(){
    return car_status.status;
}

} //namespace xqserial_server
