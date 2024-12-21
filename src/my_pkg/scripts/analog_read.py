#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Int16
import json

class AnalogReadNode:
    def __init__(self):
        rospy.init_node('analogRead_node')
        rospy.logwarn("analogRead_node started")

        self.pub_A0 = rospy.Publisher('analogRead_A0', Int16, queue_size=10)
        self.pub_A1 = rospy.Publisher('analogRead_A1', Int16, queue_size=10)
        self.pub_A2 = rospy.Publisher('analogRead_A2', Int16, queue_size=10)
        self.pub_A3 = rospy.Publisher('analogRead_A3', Int16, queue_size=10)
        self.pub_A4 = rospy.Publisher('analogRead_A4', Int16, queue_size=10)
        self.pub_A5 = rospy.Publisher('analogRead_A5', Int16, queue_size=10)

        rospy.Subscriber('analog_read', String, self.callback)
        
        self.values = {"A0": 0, "A1": 0, "A2": 0, "A3": 0, "A4": 0, "A5": 0}

        self.rate = rospy.Rate(10)  # 10 Hz

    def callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.values = {key: data.get(key, 0) for key in self.values.keys()}
        except json.JSONDecodeError:
            rospy.logerr("Failed to decode JSON from analog_read")

    def run(self):
        while not rospy.is_shutdown():
            self.pub_A0.publish(self.values["A0"])
            self.pub_A1.publish(self.values["A1"])
            self.pub_A2.publish(self.values["A2"])
            self.pub_A3.publish(self.values["A3"])
            self.pub_A4.publish(self.values["A4"])
            self.pub_A5.publish(self.values["A5"])

            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = AnalogReadNode()
        node.run()
    except rospy.ROSInterruptException:
        pass

