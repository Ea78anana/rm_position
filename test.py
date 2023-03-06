#!/usr/bin/env python

# Author: Amir Hossein Ebrahimnezhad
# Description: Python code for simulating Anafi Takeoff on Sphinx 


import rospy
from geometry_msgs.msg import PoseStamped

import olympe
import olympe_deps as od
from olympe.messages.skyctrl.CoPiloting import setPilotingSource
from olympe.enums.ardrone3.Piloting import MoveTo_Orientation_mode
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

class TakeOff_Class:

    def __init__(self):
        self.DRONE_IP = os.environ.get("DRONE_IP", "10.202.0.1")
        self.drone = olympe.Drone(self.DRONE_IP)



    def test_takeoff(self):
        os.system('clear')
        self.drone(GPSFixStateChanged(fixed=1, _timeout=10, _policy="check_wait"))
        StartingPosition = self.drone.get_state(GpsLocationChanged)
        print("StartingPosition:", StartingPosition)
        self.drone(FlyingStateChanged(state="hovering", _policy="check")  | FlyingStateChanged(state="flying", _policy="check") | (GPSFixStateChanged(fixed=1, _timeout=10, _policy="check_wait")>> (TakeOff(_no_expect=True)& FlyingStateChanged(state="hovering", _timeout=10, _policy="check_wait")))).wait()
        print("%%%%%%%%% Drone finished taking off %%%%%%%%%%%%%%%%%%%%%")
        #Get self.drone position 
        TakeOffLocation = self.drone.get_state(GpsLocationChanged)
        print(TakeOffLocation)
        d = input("hit enter to continue: ")
        #### Moveby Waypoint 1 ####
        dX = 0.5
        dY = 0
        dZ = 0 # to go up is negative

        self.drone(
            moveBy(dX,dY,dZ,dPsi=0)
            #>> PCMD(1, 0, 0, 0, 0, 0)
            # Wait for self.drone to be in hovering state
            >> FlyingStateChanged(state="hovering", _timeout=5)
        ).wait().success()
        print("%%%%%% Finished moving to WP 1 (wait 3 sec before getting position) %%%%")
        # Wait a little, for some reason the self.drone may still be moving ??
        time.sleep(3)
        # Get self.drone position 
        WayPoint1_Location = self.drone.get_state(GpsLocationChanged)
        print(WayPoint1_Location)
        d = input("hit enter to continue: ")
        Lat = TakeOffLocation["latitude"]
        Longi = TakeOffLocation["longitude"]
        Alt = 0
        self.drone(
            moveTo(Lat, Longi, Alt, MoveTo_Orientation_mode.NONE, 0.0) 
            >> moveToChanged(orientation_mode=MoveTo_Orientation_mode.NONE, status='DONE', _policy='check_wait')
            >> FlyingStateChanged(state="hovering", _timeout=5)
        ).wait()
        print("%%%% Finished moving home (wait 3 sec before getting position) %%%%")
        # Wait a little, for some reason the self.drone is still moving ??
        time.sleep(4)
        # Get self.drone position 
        drone_location = self.drone.get_state(GpsLocationChanged)
        print(drone_location)
        d = input("hit enter to continue: ")
        self.drone(
            Landing()
            >> FlyingStateChanged(state="landed", _timeout=5)
            ).wait()
        print("%%%%%%%%%%%%%%%%%%%% Finished landing %%%%%%%%%%%%%%%%%")
            #assert self.drone(extended_move_by(10, 0, 0, 0, 1, 1, 1)>> FlyingStateChanged(state="hovering", _timeout=5)).wait().success()
            #assert self.drone(moveBy(-10, 0, 0, 0)>> FlyingStateChanged(state="hovering", _timeout=5)).wait().success()
            #assert self.drone(moveBy(0, 0, -10, 0)>> FlyingStateChanged(state="hovering", _timeout=5)).wait().success()
        self.drone.disconnect()

    def position_callback(self, msg):
        #self.drone = olympe.Drone(self.DRONE_IP)
        drone_location = self.drone.get_state(GpsLocationChanged)
        print("GPS location:", drone_location)
        time.sleep(1)
        print("Optitrack location:", msg)

    def connection(self):
        self.drone.connect()

    def listener(self):
        rospy.init_node('drone_takeoff_node', anonymous=False)
        #rospy.loginfo("Takeoff Initiated!")
        sub = rospy.Subscriber('/natnet_ros/Drone1/pose', PoseStamped, takeoff.position_callback)
        rospy.spin()

if __name__ == "__main__":
    takeoff = TakeOff_Class()
    takeoff.connection()
    takeoff.test_takeoff()
