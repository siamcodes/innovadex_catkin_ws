#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16, String
import tkinter as tk
import time

class CircleDetectionGUI:
	def __init__(self, root):
		self.root = root
		self.root.title("Circle Detection Status")
		self.root.geometry("300x300")
		
		self.counter = 0  # Initialize counter
		self.last_update_time = 0  # Track the last update time
		self.update_interval = 1  # Update interval in seconds
		
		self.servo_position = None  # Current servo position
		self.servo_position_time = 0  # Time when servo position was last updated
		self.servo_delay = 2  # Delay for servo position 90

		# Create and place the label to display detection status
		self.label = tk.Label(root, text="No object", fg="black", font=("Arial", 20))
		self.label.pack(pady=10)

		# Create and place the label to display the counter
		self.counter_label = tk.Label(root, text=f"Counter: {self.counter}", fg="blue", font=("Arial", 16))
		self.counter_label.pack(pady=10)
		
		# Initialize the ROS publisher
		self.pub = rospy.Publisher('/digital_write', String, queue_size=10)
		
		# ROS subscriber
		rospy.Subscriber("circle_detect_status", Int16, self.update_label)

	def update_label(self, msg):
		current_time = time.time()
		if msg.data == 1:
			self.label.config(text="Object detected", fg="green")
			# Publish a message to move the servo to position 180
			self.pub.publish("9:180")  # Command to move servo on pin 9 to position 180
			
			# Check if enough time has passed since the last update
			if current_time - self.last_update_time >= self.update_interval:
				self.increment_counter()
				self.last_update_time = current_time
		else:
			self.label.config(text="No object", fg="black")
			# Publish a message to move the servo to position 90
			self.pub.publish("9:90")  # Command to move servo on pin 9 to position 90

	def increment_counter(self):
		# Increment the counter and update the GUI
		self.counter += 1
		self.counter_label.config(text=f"Counter: {self.counter}")

def main():
	# Initialize the ROS node
	rospy.init_node('circle_detection_gui', anonymous=True)
	
	# Initialize Tkinter window
	root = tk.Tk()
	gui = CircleDetectionGUI(root)
	
	# Start Tkinter main loop
	root.mainloop()

if __name__ == "__main__":
	main()

