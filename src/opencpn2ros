#!/usr/bin/env python
# coding=utf-8

# Author : Guilherme SCHVARCZ FRANCO
# Date : 19/05/2018

import rospy
from geometry_msgs.msg import PoseStamped, Pose, PoseArray, Twist, Quaternion
from nav_msgs.msg import Path
from math import floor
from nmea_msgs.msg import Sentence
import tf
from numpy import rad2deg
import pyproj as proj

UTM30N = proj.Proj("+init=EPSG:32630")
UTM31N = proj.Proj("+init=EPSG:32631")

def latlong2utm(lat, lon):
    return UTM30N(lon, lat)

def utm2latlong(x, y):
    return UTM30N(x, y, inverse=True)

def nmeaToDeg(nmeaNro):
    return floor(nmeaNro/100.) + (nmeaNro/100.-floor(nmeaNro/100.))*100/60

def degToNmea(deg):
    deg = abs(deg)
    return floor(deg)*100. + (deg - floor(deg))*60

def checksum(msg):
    return str(hex(reduce((lambda a,b : a^b), map(ord, msg))))[2:].upper()


class NMEA_WPL:
    def __init__(self, datastring):
        parts = datastring[1:-3].split(",")
        self.latitude = nmeaToDeg(float(parts[1]))
        self.latitudeH = parts[2]
        self.longitude = nmeaToDeg(float(parts[3]))
        self.longitudeH = parts[4]

        if self.latitudeH == "S":
            self.latitude = -self.latitude
        if self.longitudeH == "W":
            self.longitude = -self.longitude


class NMEA_RTE:
    def __init__(self, datastring):
        parts = datastring[1:-3].split(",")
        self.nro_total = float(parts[1])
        self.nro       = float(parts[2])
        self.type      = parts[3]
        self.name      = parts[4]
        self.wps       = parts[5]


class OpenCPN2ROS:
    def __init__(self):
        rospy.Subscriber('nmea_sentence', Sentence, self.cb_nmeaSentence, queue_size=1000000)
        self.pub_path = rospy.Publisher('new_waypoints_mission', Path, queue_size=1000000)
        self.pts = []
        self.rtes = []
        self.loaded = False

    def cb_nmeaSentence(self, msg):
        if (msg.sentence.strip() == "") or (msg.sentence[0] != "$") or (checksum(msg.sentence[1:-3]) != msg.sentence[-2:]):
            return

        msgType = msg.sentence[1:-3].split(",")[0]
        if msgType == "ECWPL":
            print "Processing: ", msg.sentence
            if self.loaded:
                self.loaded = False
                self.pts = []
            wp = NMEA_WPL(msg.sentence)
            self.pts.append(wp)
        elif msgType == "ECRTE":
            if not self.loaded:
                self.publishWaypointList()
            self.loaded = True
            rtem = NMEA_RTE(msg.sentence)
            self.rtes.append(rtem)

    def publishWaypointList(self):
        waypoint_msg = Path()
        waypoint_msg.header.stamp = rospy.Time.now()
        waypoint_msg.header.frame_id = "mission_waypoints"

        for pt in self.pts:
            wp = PoseStamped()
            wp.pose.position.x, wp.pose.position.y = latlong2utm(pt.latitude, pt.longitude)
            waypoint_msg.poses.append(wp)

        self.pub_path.publish(waypoint_msg)

if __name__ == '__main__':
    rospy.init_node('opencpn2ros')
    OpenCPN2ROS()
    rospy.spin()
