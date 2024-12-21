#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16
from tkinter import Tk, Label

class SensorGUI:
    def __init__(self, root):
        self.root = root
        self.root.geometry("300x300")
        self.root.title("Sensor Value Display")

        # Create a Label to display the sensor value
        self.label = Label(self.root, text="Sensor Value: ", font=("Arial", 16))
        self.label.pack(pady=50)

        # Initialize ROS Node
        rospy.init_node('GUI', anonymous=True)

        # Subscribe to the ROS topic
        rospy.Subscriber('digitalRead_D2', Int16, self.callback)

    def callback(self, msg):
        # Update the label with the received sensor value
        self.label.config(text=f"Sensor Value: {msg.data}")

    def run(self):
        # Run the Tkinter main loop
        try:
            self.root.mainloop()
        except rospy.ROSInterruptException:
            pass
            

if __name__ == '__main__':
    # Create the Tkinter root window
    root = Tk()
    gui = SensorGUI(root)
    gui.run()

