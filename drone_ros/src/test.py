import olympe
import olympe.media
import numpy as np
import olympe_deps as od
import matplotlib.pyplot as plt 
from olympe.messages.skyctrl.CoPiloting import setPilotingSource
from olympe.enums.ardrone3.Piloting import MoveTo_Orientation_mode
from olympe.enums.ardrone3.GPSSettings import HomeType_Type
from olympe.messages.ardrone3.PilotingState import moveToChanged, FlyingStateChanged, PositionChanged, AttitudeChanged
from olympe.messages.ardrone3.GPSSettingsState import GPSFixStateChanged
from olympe.messages.ardrone3.GPSSettingsState import HomeChanged, ResetHomeChanged
from olympe.messages.ardrone3.GPSSettings import HomeType, SetHome, ResetHome
from olympe.messages.ardrone3.PilotingSettingsState import MaxTiltChanged
from olympe.messages.ardrone3.GPSState import HomeTypeChosenChanged
from olympe.messages.ardrone3.SpeedSettingsState import MaxPitchRollRotationSpeedChanged, MaxRotationSpeedChanged
from olympe.messages.ardrone3.PilotingState import GpsLocationChanged, AltitudeChanged
from olympe.messages.ardrone3.Piloting import TakeOff, moveBy, Landing, moveTo, Circle, PCMD
import os
import re
import requests
import shutil
import tempfile
import xml.etree.ElementTree as ET
import time
import math

ANAFI_IP = "192.168.42.1"
ANAFI1_IP = "192.168.44.1"
ANAFI2_IP = "192.168.45.1"
SPHINX_IP = "10.202.0.1"

def takeoff():
    #drone1 = olympe.Drone(ANAFI1_IP)
    drone2 = olympe.Drone(ANAFI_IP)
    #drone1.connect()
    drone2.connect()
    #drone2(TakeOff()).wait().success()
    #drone1(Landing()).wait().success()
    drone2(Landing()).wait().success()

    #drone1.disconnect()
    drone2.disconnect()


if __name__ == "__main__":
    takeoff()