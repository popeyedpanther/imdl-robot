#!/bin/sh
# launcher.sh
# navigate to home directory, then to this directory, then execute the 
# python script, then back home.

cd /
cd home/pi/Documents/IMDL_Robot/Raspberry_Node
sudo python Main.py
cd /
