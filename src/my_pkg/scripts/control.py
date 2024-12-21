#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16, String, Bool

def callback(data):
	pub = rospy.Publisher('digital_write', String,  queue_size=10)
    	
    	########################################
    	
    	########################################
	if data.data == 1:
    		pub.publish("9:90")
	elif data.data == 0:
    		pub.publish("9:180")
    		rospy.loginfo("Sensor was deteced")

def control_node():
	rospy.init_node('control', anonymous=True)
	rospy.Subscriber('digitalRead_D2', Int16, callback)
	rospy.spin()

if __name__ == '__main__':
	try:
    		control_node()
    		
	except rospy.ROSInterruptException:
    		pass
