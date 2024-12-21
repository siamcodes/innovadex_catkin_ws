#!/usr/bin/env python3

import tkinter as tk
import subprocess
import os
import signal

class SimpleApp:
	def __init__(self, root):
		self.root = root
		self.root.title("Start/Stop App")
		
		# Create Start button
		self.start_button = tk.Button(root, text="Start", command=self.start_action)
		self.start_button.pack(pady=10)

		# Create Stop button
		self.stop_button = tk.Button(root, text="Stop", command=self.stop_action)
		self.stop_button.pack(pady=10)
		
		# Initialize process list
		self.processes = []

	def start_action(self):
		# Start ROS nodes
		self.processes.append(subprocess.Popen(["rosrun", "my_pkg", "box_detector.py"]))
		self.processes.append(subprocess.Popen(["rosrun", "my_pkg", "circle_detector.py"]))
		self.processes.append(subprocess.Popen(["rosrun", "my_pkg", "GUI_BOX.py"]))
		self.processes.append(subprocess.Popen(["rosrun", "my_pkg", "GUI_Circle.py"]))
		# Add code to handle what happens when the Start button is pressed


	def stop_action(self):
		print("Stop button pressed")
		# Terminate all processes
		for proc in self.processes:
			if proc.poll() is None:  # Check if process is still running
				proc.terminate()    # Send termination signal
				try:
					proc.wait(timeout=0.5)  # Wait for the process to terminate
				except subprocess.TimeoutExpired:
					proc.kill()  # Force kill if process does not terminate in time
		# Clear process list
		self.processes = []

if __name__ == "__main__":
	root = tk.Tk()
	app = SimpleApp(root)
	root.mainloop()


