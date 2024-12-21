#!/usr/bin/env python3

import rospy
from tkinter import Tk, Button, Label

# Define a class for the GUI
class SimpleGUI:
    def __init__(self, root):
        self.root = root
        self.root.geometry('300x300')  # Set window size to 300x300 pixels
        self.root.title("ROS Noetic Tkinter GUI")

        # Create a button and attach a callback function
        self.button = Button(root, text="Start", command=self.on_button_Start)
        self.button.pack(pady=20)  # Place the button with some padding
        self.button2 = Button(root, text="Stop", command=self.on_button_Stop)
        self.button2.pack(pady=30)  # Place the button with some padding
        self.label = Label(root, text="Stand by ", fg="black", font=("Arial", 20))
        self.label.pack(pady=50)
       

    def on_button_Start(self):
        self.label.config(text="On Process", fg="Blue")
        rospy.loginfo("On Process")
        
    def on_button_Stop(self):
        self.label.config(text="End Task", fg="Blue")
        rospy.loginfo("End Task")
        

def main():
    # Initialize ROS node
    rospy.init_node('gui_node', anonymous=True)

    # Create Tkinter root window
    root = Tk()
    
    # Instantiate the GUI class
    gui = SimpleGUI(root)
    
    # Start the Tkinter main loop
    try:
        root.mainloop()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()

