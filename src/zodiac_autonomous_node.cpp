#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <ros/ros.h>
#include <nmea_msgs/Sentence.h>

using namespace std;

struct NMEA_WPL
{
  NMEA_WPL(string datastring)
  {
    std::stringstream parts(datastring);
    string part;

    std::getline(parts,part,',');

    std::getline(parts,part,',');
    this->x = std::stod(part);
    std::getline(parts,part,',');
    this->xH = part;

    std::getline(parts,part,',');
    this->y = std::stod(part);
    std::getline(parts,part,',');
    this->yH = part;

    this->name = part;
  }

  string toString()
  {
    std::stringstream ss;
    ss << this->name << ": " << this->x << ", " << this->y;
    return ss.str();
  }

  double x, y;
  string xH, yH, name;
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


class ZodiacAutonomous
{
public:

  ZodiacAutonomous () : loaded(true)
  {
    ros::NodeHandle nodeLocal("~");

    std::string ns = ros::this_node::getNamespace();

    sub1 = n.subscribe("/nmea_sentence", 1000000, &ZodiacAutonomous::nmeaSentenceCallback, this);
    pub1 = n.advertise<nmea_msgs::Sentence>("/nmea_sentence",0);
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
        publishBoatPose(sentence_msg);
      }
      NMEA_WPL wp(sentence_msg.sentence);
      pts.push_back(wp);
      cout << wp.toString() << endl;
    }
    else if(part.compare("$ECRTE") == 0)
    {
      loaded = true;
      NMEA_RTE rtem(sentence_msg.sentence);
      rte.push_back(rtem);
      cout << rtem.toString() << endl;
    }
  }

  void publishBoatPose(nmea_msgs::Sentence old_msg)
  {
    std::stringstream msg_ss, msg_final;
    string msg_s;
    //    msg_ss << "GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W";
    msg_ss << "GPGGA,123519,4807.038,N,01131.000,E,1,10,0.9,545.4,M,46.9,M,,";
//    msg_ss  << "GPRMC,123519,A," << 4807.038 + rand() %200 << ",N," << 1131.000 + rand() %200 << ",E,022.4,084.4,230394,003.1,W";
    msg_s = msg_ss.str();

    msg_final << "$" << msg_s << "*" << std::hex << (checksum(msg_s.c_str()) & 255);
    nmea_msgs::Sentence msg = old_msg;
    msg.header.stamp = ros::Time::now();
    msg.sentence = msg_final.str();
    pub1.publish(msg);
  }

  int checksum(const char *str)
  {
    int i = 0;
    int ret = 0;
    while(str[i] != '\0')
    {
      ret ^= str[i];
      i++;
    }
    return ret;
  }

private:
  ros::NodeHandle n;
  ros::Subscriber sub1;
  ros::Publisher pub1;
  vector<NMEA_WPL> pts;
  vector<NMEA_RTE> rte;
  bool loaded;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "zodiac_autonomous_node");

  ZodiacAutonomous mZodiacAutonomous;

  ros::spin();
}
