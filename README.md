OpenCPN2ROS
===========
![ROS](https://img.shields.io/badge/ROS-Kinetic--Kame-green.svg)

This project aims to create a link between OpenCPN and ROS throughout a serial port and NMEA standard.

From OpenCPN point-of-view, ROS is are two gps ports:
  * `/dev/ttyVUSB0` - where OpenCPN should publish the waypoints to ROS;
  * `/dev/ttyVUSB3` - where OpenCPN receive receive ROS pose and orientation.


## Prerequisites
Install all deps:
```
sudo apt-get install ros-kinetic-nmea-navsat-driver ros-kinetic-nav-msgs socat python-pyproj python-numpy
```


## How to use

1. Create the virtual ports.

```
sudo ./script/create_ports.bash
```

2. Configure the ports at OpenCPN:

![Configure ttyVUSB0.](https://github.com/schvarcz/opencpn2ros/blob/master/docs/VUSB0.png?raw=true)
![Configure ttyVUSB1.](https://github.com/schvarcz/opencpn2ros/blob/master/docs/VUSB3.png?raw=true)

3. Launch the ROS interface:

```
roslaunch opencpn3ros opencpn_interface.launch
```

## ROS Nodes
### opencpn2ros
Read the waypoints sent by OpenCPN and publish to ROS as a nav_msgs/Path.

#### Subscribed topics
`nmea_msgs/Sentence` - nmea_sentence: NMEA Message to be analysed.
#### Published topics
`nav_msgs/Path` - new_waypoints_mission: Path to be followed by the robot.

___

### fake_gps
Reads the robot pose and publish into OpenCPN.
#### Subscribed topics
`geometry_msgs/PoseStamped` - pose: Robot's current pose.
#### Published topics
`nmea_msgs/Sentence` - nmea_sentence: NMEA Message to be sent to OpenCPN.

___

### nmea_topic_virtual_serial_reader
Reads serial port `/dev/ttyVUSB0` for incoming messages.
#### Published topics
`nmea_msgs/Sentence` - nmea_sentence: NMEA Message to be analysed.

___

### nmea_topic_virtual_serial_writer
Writes into serial port `/dev/ttyVUSB3` all outgoing messages.
#### Subscribed topics
`nmea_msgs/Sentence` - nmea_sentence: NMEA Message to be sent to OpenCPN.
