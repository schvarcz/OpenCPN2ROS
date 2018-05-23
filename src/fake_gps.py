#!/usr/bin/env python
# coding=utf-8

# Author : Guilherme SCHVARCZ FRANCO
# Date : 19/05/2018

import rospy
from geometry_msgs.msg import PoseStamped, Pose, Twist, Quaternion
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

def quaternionToQuaternionMsg(quat):
    q = Quaternion()
    q.x = quat[0]
    q.y = quat[1]
    q.z = quat[2]
    q.w = quat[3]
    return q

def quaternionMsgToQuaternion(quat):
    return ( quat.x, quat.y, quat.z, quat   .w )

def getYaw(pose):
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternionMsgToQuaternion(pose.pose.orientation))
    return yaw

def nmeaToDeg(nmeaNro):
    return floor(nmeaNro/100.) + (nmeaNro/100.-floor(nmeaNro/100.))*100/60

def degToNmea(deg):
    deg = abs(deg)
    return floor(deg)*100. + (deg - floor(deg))*60

def latH(lat):
    if lat<0:
        return "S"
    return "N"

def lonH(lon):
    if lon<0:
        return "W"
    return "E"

def checksum(msg):
    return str(hex(reduce((lambda a,b : a^b), map(ord, msg))))[2:]

def secsToNmeaTime(secs):
    hours = floor(secs/3600)
    secs -= hours*3600
    hours = (hours/24. - floor(hours/24.))*24
    minutes = floor(secs/60)
    secs -= minutes*60
    return "{0:02}{1:02}{2:02}".format(int(hours), int(minutes), int(floor(secs)))

class FakeGPS:
    def __init__(self):
        rospy.Subscriber('pose', PoseStamped, self.cb_pose, queue_size=1000000)
        self.pub_nmea = rospy.Publisher('nmea_sentence', Sentence, queue_size=10000)
        self.rate = rospy.Rate(rospy.get_param('~rate', 3))
        self.cur_pose = None

    def cb_pose(self, pose_msg):
        self.cur_pose = pose_msg

    def sendPose(self):
        if self.cur_pose == None:
            return
        lon, lat = utm2latlong(self.cur_pose.pose.position.x, self.cur_pose.pose.position.y)
        timeInSecs = self.cur_pose.header.stamp.to_sec()
        time = secsToNmeaTime(timeInSecs)
        yaw = -rad2deg(getYaw(self.cur_pose))+90

        print "time: ", time, ", sec: ",timeInSecs
        msg_comp = "GPGGA," + time + "," + "{0:08.3f}".format(degToNmea(lat)) + "," + latH(lat) + "," + "{0:09.3f}".format(degToNmea(lon)) + "," + lonH(lon) + "," + "4,10,0," + str(self.cur_pose.pose.position.z) + ",M," + str(self.cur_pose.pose.position.z) + ",M,,"
        self.sendSentence(msg_comp)
        msg_comp = "GPHDT," + str(yaw) + ",T"
        self.sendSentence(msg_comp)

        # msg_comp = "GPRMC," + time + ",A," + "{0:08.3f}".format(degToNmea(lat)) + "," + latH(lat) + "," + "{0:09.3f}".format(degToNmea(lon)) + "," + lonH(lon) + "," + "0," + str(yaw) + ",230518,003.1,W"
        # self.sendSentence(msg_comp)
        # msg_comp = "GPHDM," + str(yaw) + ",T"
        # self.sendSentence(msg_comp)
        # msg_comp = "GPHSC," + str(yaw) + ",T,"+ str(yaw) + ",M  "
        # self.sendSentence(msg_comp)
        # msg_comp = "HCHDG," + str(yaw) + ",,,7.1,W "
        # self.sendSentence(msg_comp)

    def sendSentence(self, msg):
        sentence = Sentence()
        sentence.header.stamp = rospy.Time.now()
        sentence.sentence = "$" + msg + "*" + checksum(msg)
        self.pub_nmea.publish(sentence)

    def run(self):
        while not rospy.is_shutdown():
            self.sendPose()
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('fake_gps')
    FakeGPS().run()
