#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Int16
import json

class DigitalReadNode:
    def __init__(self):
        rospy.init_node('digitalRead_node')
        rospy.logwarn("digitalRead_node started")

        self.pub_D2 = rospy.Publisher('digitalRead_D2', Int16, queue_size=10)
        self.pub_D4 = rospy.Publisher('digitalRead_D4', Int16, queue_size=10)

        rospy.Subscriber('digital_read', String, self.callback)

        self.values = {"D2": 0, "D4": 0}

        self.rate = rospy.Rate(10)  # 10 Hz

    def callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.values = {key: data.get(key, 0) for key in self.values.keys()}
        except json.JSONDecodeError:
            rospy.logerr("Failed to decode JSON from digital_read")

    def run(self):
        while not rospy.is_shutdown():
            self.pub_D2.publish(self.values["D2"])
            self.pub_D4.publish(self.values["D4"])

            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = DigitalReadNode()
        node.run()
    except rospy.ROSInterruptException:
        pass

