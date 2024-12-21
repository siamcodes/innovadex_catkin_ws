#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool

def led_control(state):
    pub = rospy.Publisher('led_control', Bool, queue_size=10)
    rospy.init_node('led_controller', anonymous=True)
    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        rospy.loginfo(f"LED State: {'ON' if state else 'OFF'}")
        pub.publish(state)
        rate.sleep()

if __name__ == '__main__':
    try:
        # ตั้งค่า True เพื่อเปิด LED และ False เพื่อปิด
        led_control(False)  # เปลี่ยนค่าเป็น False เพื่อปิด LED
    except rospy.ROSInterruptException:
        pass

