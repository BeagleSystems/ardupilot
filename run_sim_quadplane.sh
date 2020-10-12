#!/bin/bash

cd /media/sf_BeagleSystems/ardupilot/Tools/autotest

python sim_vehicle.py -v ArduPlane -f quadplane --console --map -L WestNetzWimbach -S 1 --out 192.168.2.45:14550
