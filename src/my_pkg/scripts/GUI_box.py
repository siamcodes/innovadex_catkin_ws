#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16
import tkinter as tk

class BoxDetectionGUI:
	def __init__(self, root):
    		self.root = root
    		self.root.title("Box Detection Status")
    		self.root.geometry("300x300")
   	 
    		self.label = tk.Label(root, text="No object", fg="black", font=("Arial", 20))
    		self.label.pack(pady=100)
   	 
    		# ROS subscriber
    		rospy.Subscriber("box_detect_status", Int16, self.update_label)

	def update_label(self, msg):
    		if msg.data == 1:
        		self.label.config(text="Object detected", fg="green")
    		else:
        		self.label.config(text="No object", fg="black")



def main():
	# Initialize the ROS node
	rospy.init_node('box_detection_gui', anonymous=True)
    
	# Initialize Tkinter window
	root = tk.Tk()
	gui = BoxDetectionGUI(root)
	
    
	# Start Tkinter main loop in a non-blocking way
	root.after(100, lambda: None)
	root.mainloop()

if __name__ == "__main__":
	main()


