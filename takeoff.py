#!/usr/bin/env python

# Author: Amir Hossein Ebrahimnezhad
# Description: Python code for simulating Anafi Takeoff on Sphinx 


import rospy

import olympe
import os
import time
from olympe.messages.ardrone3.Piloting import TakeOff, Landing, moveBy, moveTo,Circle
from olympe.messages.move import extended_move_by
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged, SpeedChanged

class TakeOff_Class:

    def __init__(self):
        self.DRONE_IP = os.environ.get("DRONE_IP", "10.202.0.1")



    def test_takeoff(self):
        drone = olympe.Drone(self.DRONE_IP)
        drone.connect()
        assert drone(TakeOff() >> FlyingStateChanged(state="hovering", _timeout=5)).wait().success()
        assert drone(extended_move_by(10, 10, 0, 30, 1, 1, 1)>> FlyingStateChanged(state="hovering", _timeout=5)).wait().success()
        assert drone(moveBy(-10, 0, 0, 0)>> FlyingStateChanged(state="hovering", _timeout=5)).wait().success()
        assert drone(moveBy(0, 0, -10, 0)>> FlyingStateChanged(state="hovering", _timeout=5)).wait().success()
        print("current state:",drone.get_state(SpeedChanged))
        assert drone(Landing()).wait().success()
        drone.disconnect()

    def position_callback(msg):
        pass

    def listener(data):
        pass

if __name__ == "__main__":

    rospy.init_node('drone_takeoff_node', anonymous=False)
    rospy.loginfo("Takeoff Initiated!")

    take_off = TakeOff_Class()

    take_off.test_takeoff()
