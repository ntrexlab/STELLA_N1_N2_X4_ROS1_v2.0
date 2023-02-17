#! /usr/bin/env python

import serial
import time
import rospy
from geometry_msgs.msg import Twist

if __name__=="__main__":
	serial_port = serial.Serial(
		port="/dev/BT",
    		baudrate=9600,
    		bytesize=serial.EIGHTBITS,
    		parity=serial.PARITY_NONE,
    		stopbits=serial.STOPBITS_ONE,
	)

	# Wait a second to let the port initialize
	time.sleep(1)
	
	rospy.init_node('stella_teleop_bluetooth_node')
	pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
	rate = rospy.Rate(10)
	target_linear_vel   = 0.0
	target_angular_vel  = 0.0

	try:
    		while True:
        		if serial_port.inWaiting() > 0:
            			data = serial_port.readline()
            			a = data[0:5]
            			a = str(a)
				a = a.split("/")
				
				if a[0] == 'F':
					target_linear_vel = float(a[1]) * 1
				elif a[0] == 'B':
					target_linear_vel = float(a[1]) * -1
				elif a[0] == 'L':
					target_angular_vel = float(a[1]) * 1.57
				elif a[0] == 'R':
					target_angular_vel = float(a[1]) * -1.57
				elif a[0] == 'S':
					target_linear_vel  = 0.0
					target_angular_vel = 0.0
									
			else:
				target_linear_vel  = 0.0
				target_angular_vel = 0.0		
		
			twist = Twist()
			
			twist.linear.x = target_linear_vel
			twist.linear.y = 0.0
			twist.linear.z = 0.0

			twist.angular.x = 0.0
			twist.angular.y = 0.0
			twist.angular.z = target_angular_vel
			

			pub.publish(twist)
			rate.sleep()

	except KeyboardInterrupt:
    		print("Exiting Program")
	except Exception as exception_error:
    		print("Error occurred. Exiting Program")
    		print("Error: " + str(exception_error))
	finally:
    		serial_port.close()
    	pass
