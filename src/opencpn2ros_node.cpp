#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <ros/ros.h>
#include <nmea_msgs/Sentence.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>

using namespace std;

float nmeaToDeg(float nmeaNro)
{
  return floor(nmeaNro/100.) + (nmeaNro/100.-floor(nmeaNro/100.))*100/60;
}

float degToNmea(float deg)
{
  return floor(deg)*100. + (deg - floor(deg))*60;
}

struct NMEA_WPL
{
  NMEA_WPL(string datastring)
  {
    std::stringstream parts(datastring);
    string part;

    std::getline(parts,part,',');

    std::getline(parts,part,',');
    this->latitude = nmeaToDeg(std::stod(part));
    std::getline(parts,part,',');
    this->latitudeH = part;
    if(this->latitudeH.compare("S") == 0)
      this->latitude = -this->latitude;

    std::getline(parts,part,',');
    this->longitude = nmeaToDeg(std::stod(part));
    std::getline(parts,part,',');
    this->longitudeH = part;
    if(this->longitudeH.compare("W") == 0)
      this->longitude = -this->longitude;

    this->name = part;
  }

  string toString()
  {
    std::stringstream ss;
    ss << this->name << ": " << this->latitude << ", " << this->longitude;
    return ss.str();
  }

  double latitude, longitude;
  string latitudeH, longitudeH, name;
};

struct NMEA_RTE
{
  NMEA_RTE(string datastring)
  {
    std::stringstream parts(datastring);
    string part;

    std::getline(parts,part,',');

    std::getline(parts,part,',');
    this->nro_total = std::stod(part);
    std::getline(parts,part,',');
    this->nro = std::stod(part);

    std::getline(parts,part,',');
    this->type = part;
    std::getline(parts,part,',');
    this->name = part;

    std::getline(parts,part,',');
    this->wps = part; //should improve this part
  }

  string toString()
  {
    std::stringstream ss;
    ss << this->nro << "/" << this->nro_total << " " <<this->name << ": " << this->wps;
    return ss.str();
  }

  double nro, nro_total;
  string type, name, wps;
};


class OpenCPN2ROS
{
public:

  OpenCPN2ROS () : loaded(true)
  {
    ros::NodeHandle nodeLocal("~");

    std::string ns = ros::this_node::getNamespace();

    sub1 = n.subscribe("nmea_sentence", 1000000, &OpenCPN2ROS::nmeaSentenceCallback, this);
    sub2 = n.subscribe("pose", 1000000, &OpenCPN2ROS::poseCallback, this);
    pub1 = n.advertise<nmea_msgs::Sentence>("nmea_sentence",1000);
    // pub2 = n.advertise<geometry_msgs::PoseArray>("new_waypoints_mission", 1000);
    pub2 = n.advertise<nav_msgs::Path>("new_waypoints_mission", 1000);
  }

  void nmeaSentenceCallback(const nmea_msgs::Sentence sentence_msg)
  {
    cout << sentence_msg.sentence << endl;

    std::stringstream parts(sentence_msg.sentence);
    string part;

    std::getline(parts,part,',');

    if(part.compare("$ECWPL") == 0)
    {
      if(loaded)
      {
        loaded = false;
        pts.clear();
      }
      NMEA_WPL wp(sentence_msg.sentence);
      pts.push_back(wp);
    }
    else if(part.compare("$ECRTE") == 0)
    {
      if(!loaded)
        publishWaypointList();
      loaded = true;
      NMEA_RTE rtem(sentence_msg.sentence);
      rte.push_back(rtem);
    }
  }

  // void publishWaypointList()
  // {
  //   geometry_msgs::PoseArray waypoint_msg;
  //   waypoint_msg.header.stamp = ros::Time::now();
  //   waypoint_msg.header.frame_id = "mission_waypoints";
  //
  //   int i = 1;
  //   for(auto pt : pts)
  //   {
  //     geometry_msgs::Pose wp;
  //     wp.position.x = pt.longitude;
  //     wp.position.y = pt.latitude;
  //     waypoint_msg.poses.push_back(wp);
  //   }
  //   pub2.publish(waypoint_msg);
  // }

  void publishWaypointList()
  {
    nav_msgs::Path waypoint_msg;
    waypoint_msg.header.stamp = ros::Time::now();
    waypoint_msg.header.frame_id = "mission_waypoints";

    int i = 1;
    for(auto pt : pts)
    {
      geometry_msgs::PoseStamped wp;
      wp.pose.position.x = pt.longitude;
      wp.pose.position.y = pt.latitude;
      waypoint_msg.poses.push_back(wp);
    }
    pub2.publish(waypoint_msg);
  }

  void poseCallback(const geometry_msgs::PoseStamped pose_msg)
  {
    std::stringstream msg_comp;
    msg_comp << "GPGGA,123519," << degToNmea(pose_msg.pose.position.y) << (pose_msg.pose.position.y>0?",N,":",S,")  << degToNmea(pose_msg.pose.position.x) << (pose_msg.pose.position.x>0?",E,":",W,") << "4,10,0," << pose_msg.pose.position.z << ",M," << pose_msg.pose.position.z << ",M,,";
    nmea_msgs::Sentence msg;
    msg.header.stamp = ros::Time::now();
    msg.sentence = packMessage(msg_comp.str());
    // pub1.publish(msg);
  }

  //Just keeping these fucntions around in case of nmea messages directly from GPS do not work.
  void publishBoatPose()
  {
    nmea_msgs::Sentence msg;
    msg.header.stamp = ros::Time::now();
    msg.sentence = packMessage("GPGGA,123519,4807.038,N,01131.000,E,1,10,0.9,545.4,M,46.9,M,,");
    pub1.publish(msg);
  }

  string packMessage(string message)
  {
    std::stringstream msg_final;
    msg_final << "$" << message << "*" << std::hex << (checksum(message.c_str()) & 255);
    return msg_final.str();
  }

  int checksum(const char *str)
  {
    int i = 0, ret = 0;

    while(str[i])
      ret ^= str[i++];

    return ret;
  }

private:
  ros::NodeHandle n;
  ros::Subscriber sub1, sub2;
  ros::Publisher pub1, pub2;
  vector<NMEA_WPL> pts;
  vector<NMEA_RTE> rte;
  bool loaded;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "opencpn2ros");

  OpenCPN2ROS mOpenCPN2ROS;

  ros::spin();

 // ros::Rate rate(1);
 // while(rate.sleep())
 // {
 //   mOpenCPN2ROS.publishBoatPose();
 //   ros::spinOnce();
 // }
}
