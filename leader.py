#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from pymavlink import mavutil
import utils.drone
import time

vehicle = connect('127.0.0.1:14560', wait_ready=True, baud=115200)


vehicle.close()