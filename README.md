# xqserial_server
mongoose motor driver and low level ROS api package
    
## input topic
      name                                  type        
    /cmd_vel                         geometry_msgs/Twist
      
## output topic
       name                                 type                 frate
    /mongoose_serial_server/Odom            nav_msgs/Odometry            50hz
    /mongoose_serial_server/Pose2D          geometry_msgs/Pose2D         50hz
    /mongoose_serial_server/Power           std_msgs/Float64             50hz
    /mongoose_serial_server/StatusFlag      std_msgs/Int32               50hz
    /mongoose_serial_server/Twist           geometry_msgs/Twist          50hz
    /tf                              odom-->base_footprint        50hz
    /tf_static                       base_footprint-->base_link   100hz
    
## input param   
       name                            default
    port                             /dev/ttyUSB0
    baud                               115200
    wheel_separation                    0.37
    wheel_radius                        0.06
    debug_flag                          false
    max_speed                           2.0
    cmd_topic                           cmd_vel

## Usage:
### download to xiaoqiang ros workspace
```
cd ~/catkin_ws/src
git clone https://github.com/lwc3258/MongooseSerialServer.git
cd ..
catkin_make
```
### Quickstart
```
roslaunch mongoose_serial_server mongoose_serial_server.launch
```
    

Mongoose robot motor driver and ros interface for driver.
## Usage：

#### install
```
cd ~/catkin_ws/src
git clone https://github.com/lwc3258/MongooseSerialServer.git
cd ..
catkin_make
```
### 直接启动
```
roslaunch mongoose_serial_server mongoose_serial_server.launch
```
