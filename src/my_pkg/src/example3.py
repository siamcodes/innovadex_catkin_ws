#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, String

# ตัวแปร global
value = 0.0
multiplier = 1.0

def input_callback(msg):
    global multiplier
    # ปรับ multiplier ตามข้อความที่ได้รับจาก topic "input"
    if msg.data == "up":
        multiplier = 100.0
    elif msg.data == "down":
        multiplier = 0.01

def main():
    global value, multiplier
    
    # เริ่มต้น node ด้วยชื่อ "center" และตั้งค่าไม่ให้เป็น anonymous
    rospy.init_node('center', anonymous=False)
    
    # ตั้งค่า publisher และ subscriber
    pub = rospy.Publisher('output', Float32, queue_size=10)
    rospy.Subscriber('input', String, input_callback)
    
    # กำหนดอัตราการ publish ที่ 1Hz
    rate = rospy.Rate(1)
    
    while not rospy.is_shutdown():
        # ส่งค่า value ที่คูณด้วย multiplier ไปยัง topic "output"
        pub.publish(value * multiplier)
        
        # เพิ่มค่า value ทีละ 1.0
        value += 1.0
        
        # หยุดชั่วคราวเพื่อให้สอดคล้องกับอัตรา 1Hz
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

