#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16,String, Bool
from tkinter import Tk, Label, Button


class SensorGUI:
    def __init__(self, root):
        self.num = 0
        self.root = root
        self.root.geometry("300x300")
        self.root.title("Innovedex2024")

        # Create a Label to display the sensor value
        self.button = Button(root, text="Start", command=self.on_button_Start)
        self.button.pack(pady=10) 
        self.button2 = Button(root, text="Stop", command=self.on_button_Stop)
        self.button2.pack(pady=10) 
        
        self.label = Label(root, text="Stand by ", fg="black", font=("Arial", 20))
        self.label.pack(pady=10)
        self.label1 = Label(self.root, text="Sensor Value: ", font=("Arial", 20))
        self.label1.pack(pady=10)
        self.label2 = Label(self.root, text="Count Object: ", font=("Arial", 20))
        self.label2.pack(pady=10)


        # Initialize ROS Node
        rospy.init_node('GUI', anonymous=True)

        # Subscribe to the ROS topic
        rospy.Subscriber('digitalRead_D2', Int16, self.callback)
        rospy.Subscriber('digitalRead_D2', Int16, self.count_callback)

    def on_button_Start(self):
     	self.label.config(text="On Process", fg="Blue")
     	rospy.loginfo("On Process")   
     	

    def on_button_Stop(self):
        self.label.config(text="End Task", fg="Red")
        rospy.loginfo("End Task")  

    def callback(self, msg):
        # Update the label with the received sensor value
        self.label1.config(text=f"Sensor Value: {msg.data}")
        if int(msg.data)==0:
        	rospy.loginfo("Sensor was deteced")
        	

    def count_callback(self, msg):
        # Update the label with the received sensor value
        if int(msg.data)==0:
        	self.num1 = int(msg.data)
        	self.num += self.num1+1
        	self.label2.config(text=f"Count: {self.num}")
        		
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

