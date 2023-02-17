#!/bin/bash

echo "remap the devices serial port(ttyUSBX, ttySX) to  ydlidar, AHRS, Motordriver, Bluetooth"
echo "devices usb connection as /dev/YDLIDAR, /dev/AHRS, /dev/MW, /dev/BT  , check it using the command : ls -l /dev|grep -e ttyUSB -e ttyS"
echo "start copy stella.rules to  /etc/udev/rules.d/"
echo "/home/odroid/catkin_ws/src/STELLA_N1_N2_X4_ROS1_v2.0/stella_bringup/stella.rules"
sudo cp /home/odroid/catkin_ws/src/STELLA_N1_N2_X4_ROS1_v2.0/stella_bringup/stella.rules  /etc/udev/rules.d
echo " "
echo "Restarting udev"
echo ""
sudo udevadm trigger
echo "finish "
