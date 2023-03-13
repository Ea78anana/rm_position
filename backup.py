#!/usr/bin/env python

# Author: Amir Hossein Ebrahimnezhad
# Description: Python code for simulating Anafi Takeoff on Sphinx 


import rospy
from geometry_msgs.msg import PoseStamped

import olympe
import olympe.media
import olympe_deps as od
from olympe.messages.skyctrl.CoPiloting import setPilotingSource
from olympe.enums.ardrone3.Piloting import MoveTo_Orientation_mode
from olympe.enums.ardrone3.GPSSettings import HomeType_Type
from olympe.messages.ardrone3.PilotingState import moveToChanged, FlyingStateChanged, PositionChanged, AttitudeChanged
from olympe.messages.ardrone3.GPSSettingsState import GPSFixStateChanged
from olympe.messages.ardrone3.GPSSettingsState import HomeChanged, ResetHomeChanged
from olympe.messages.ardrone3.GPSSettings import HomeType, SetHome, ResetHome
from olympe.messages.ardrone3.GPSState import HomeTypeChosenChanged
from olympe.messages.ardrone3.PilotingState import GpsLocationChanged, AltitudeChanged
from olympe.messages.ardrone3.Piloting import TakeOff, moveBy, Landing, moveTo, Circle, PCMD
import os
import re
import requests
import shutil
import tempfile
import xml.etree.ElementTree as ET
import time

current = PoseStamped()

ANAFI_IP = "192.168.42.1"
SPHINX_IP = "10.202.0.1"

class TakeOff_Class:

    def __init__(self):
        self.DRONE_IP = os.environ.get(SPHINX_IP)
        self.drone = olympe.Drone(ANAFI_IP)
        self.roll = 0
        self.pich = 0
        self.yaw = 0
        self.gaz = 0
        

    def test_takeoff(self):
        drone = olympe.Drone(SPHINX_IP)
        drone(GPSFixStateChanged(fixed=1, _timeout=10, _policy="check_wait"))
        drone.connect()
        os.system('clear')
        StartingPosition = drone.get_state(GpsLocationChanged)
        print("StartingPosition:", StartingPosition)
        drone(TakeOff(_no_expect=True)& FlyingStateChanged(state="hovering", _timeout=10, _policy="check_wait")).wait()
        print("%%%%%%%%% Drone finished taking off %%%%%%%%%%%%%%%%%%%%%")
        #Get drone position 
        TakeOffLocation = drone.get_state(GpsLocationChanged)
        print(TakeOffLocation)
        d = input("hit enter to continue: ")
        #### Moveby Waypoint 1 ####
        dX = 1
        dY = 0
        dZ = -0.5 # to go up is negative

        drone(
            moveBy(dX,dY,dZ,dPsi=0)
            #PCMD(1, 0, 0, 0, 0, 0)
            # Wait for drone to be in hovering state
            #>> FlyingStateChanged(state="hovering", _timeout=5)
        ).wait().success()
        print("%%%%%% Finished moving to WP 1 (wait 3 sec before getting position) %%%%")
        # Wait a little, for some reason the drone may still be moving ??
        #time.sleep(3)
        # Get drone position 
        WayPoint1_Location = drone.get_state(GpsLocationChanged)
        print(WayPoint1_Location)
        '''
        d = input("hit enter to continue: ")
        Lat = TakeOffLocation["latitude"]
        Longi = TakeOffLocation["longitude"]
        Alt = 0
        drone(
            moveTo(Lat, Longi, Alt, MoveTo_Orientation_mode.NONE, 0.0) 
            >> moveToChanged(orientation_mode=MoveTo_Orientation_mode.NONE, status='DONE', _policy='check_wait')
            >> FlyingStateChanged(state="hovering", _timeout=5)
        ).wait()
        print("%%%% Finished moving home (wait 3 sec before getting position) %%%%")
        # Wait a little, for some reason the drone is still moving ??
        time.sleep(4)
        # Get drone position 
        drone_location = drone.get_state(GpsLocationChanged)
        print(drone_location)
        drone(listener()
                    PCMD(
                        1,
                        control.roll(),
                        control.pitch(),
                        control.yaw(),
                        control.throttle(),
                        timestampAndSeqNum=0,
                    )
                )
            else:
                drone(PCMD(0, 0, 0, 0, 0, timestampAndSeqNum=0))
            time.sleep(0.05)
        '''
        d = input("hit enter to continue: ")
        drone(
            Landing()
            >> FlyingStateChanged(state="landed", _timeout=5)
            ).wait()
        print("%%%%%%%%%%%%%%%%%%%% Finished landing %%%%%%%%%%%%%%%%%")
            #assert drone(extended_move_by(10, 0, 0, 0, 1, 1, 1)>> FlyingStateChanged(state="hovering", _timeout=5)).wait().success()
            #assert drone(moveBy(-10, 0, 0, 0)>> FlyingStateChanged(state="hovering", _timeout=5)).wait().success()
            #assert drone(moveBy(0, 0, -10, 0)>> FlyingStateChanged(state="hovering", _timeout=5)).wait().success()
        drone.disconnect()

    def position_callback(self, msg):
        #self.drone = olympe.Drone(self.DRONE_IP)
        #print("Optitrack location:", msg.pose.position)
        self.MPC(msg)

    def MPC(self, data):
        current.pose.position.x = round(data.pose.position.x,2)
        current.pose.position.y = round(data.pose.position.y,2)
        current.pose.position.z = round(data.pose.position.z,2)
        print(current.pose.position)
        '''
        if (current.pose.position.x != 0.06 and current.pose.position.y != -0.08 and current.pose.position.z != 0.39):
            self.drone(PCMD(1,0,10,0,0,0))
            #time.sleep(0.05)
        else:
            self.drone(PCMD(0,0,0,0,0,0))
        '''
        if (current.pose.position.y != -0.08):
            self.drone(PCMD(1,0,1,0,0,0))
            #time.sleep(0.05)
        else:
            self.drone(PCMD(0, 0, 0, 0, 0, 0))
            takeoff.disconnection()

    def disconnection(self):
        self.drone(Landing()).wait().success()
        self.drone.disconnect()

    def connection(self):
        self.drone.connect()
        self.drone(TakeOff()).wait().success()

    def listener(self):
        rospy.init_node('drone_takeoff_node', anonymous=False)
        rospy.loginfo("Takeoff Initiated!")
        sub = rospy.Subscriber('/natnet_ros/Drone1/pose', PoseStamped, takeoff.position_callback)
        rospy.spin()

if __name__ == "__main__":
    rospy.init_node('drone_takeoff_node', anonymous=False)
    rospy.loginfo("Takeoff Initiated!")
    takeoff = TakeOff_Class()
    takeoff.connection()
    try :
        takeoff.listener()
        #time.sleep(0.05)
        #takeoff.test_takeoff()
    except rospy.ROSInterruptException:
            pass
    
