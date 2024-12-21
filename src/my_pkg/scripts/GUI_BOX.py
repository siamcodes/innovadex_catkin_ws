#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16, String
import tkinter as tk
import time

class BoxDetectionGUI:
	def __init__(self, root):
		self.root = root
		self.root.title("Box Detection Status")
		self.root.geometry("300x300")
		
		self.counter = 0  # Initialize counter
		self.last_update_time = 0  # Track the last update time
		self.update_interval = 1  # Update interval in seconds
		
		self.servo_position = None  # Current servo position
		self.servo_position_time = 0  # Time when servo position was last updated
		self.servo_delay = 2  # Delay for servo position 90

		# Create and place the label to display detection status
		self.label = tk.Label(root, text="No box", fg="black", font=("Arial", 20))
		self.label.pack(pady=10)

		# Create and place the label to display the counter
		self.counter_label = tk.Label(root, text=f"Counter: {self.counter}", fg="blue", font=("Arial", 16))
		self.counter_label.pack(pady=10)
		
		# Initialize the ROS publisher
		self.pub = rospy.Publisher('/digital_write', String, queue_size=10)
		
		# ROS subscriber
		rospy.Subscriber("box_detect_status", Int16, self.update_label)

	def update_label(self, msg):
		current_time = time.time()
		
		if msg.data == 1:
			self.label.config(text="Box detected", fg="green")
			# Publish a message to move the servo to position 0
			self.pub.publish("9:0")  # Command to move servo on pin 9 to position 0
			
			# Check if enough time has passed since the last update
			if current_time - self.last_update_time >= self.update_interval:
				self.increment_counter()
				self.last_update_time = current_time
				
			# Update the servo position
			self.servo_position = "9:0"
			self.servo_position_time = current_time
			
		else:
			self.label.config(text="No box", fg="black")
			
			# Update servo position if it's not already set to 90
			if self.servo_position != "9:90" or current_time - self.servo_position_time >= self.servo_delay:
				self.pub.publish("9:90")  # Command to move servo on pin 9 to position 90
				self.servo_position = "9:90"
				self.servo_position_time = current_time

	def increment_counter(self):
		# Increment the counter and update the GUI
		self.counter += 1
		self.counter_label.config(text=f"Counter: {self.counter}")

def main():
	# Initialize the ROS node
	rospy.init_node('box_detection_gui', anonymous=True)
	
	# Initialize Tkinter window
	root = tk.Tk()
	gui = BoxDetectionGUI(root)
	
	# Start Tkinter main loop
	root.mainloop()

if __name__ == "__main__":
	main()

