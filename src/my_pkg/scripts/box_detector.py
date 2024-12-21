#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int16
import tkinter as tk
from PIL import Image as PILImage, ImageTk
import time

class BoxDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        self.pub = rospy.Publisher("box_detect_status", Int16, queue_size=10)
        self.detected = False

        self.low_threshold = 50
        self.high_threshold = 150
        self.min_area = 1000

        self.cv_image = None

        self.setup_gui()

    def setup_gui(self):
        self.root = tk.Tk()
        self.root.title("Box Detector Parameters and Video Feed")
        self.root.geometry("550x300")

        self.canvas = tk.Canvas(self.root, width=550, height=300)
        self.canvas.pack()

        self.low_thresh_scale = tk.Scale(self.root, from_=0, to=255, orient=tk.HORIZONTAL, label="Low Canny Threshold")
        self.low_thresh_scale.set(self.low_threshold)
        self.low_thresh_scale.place(x=10, y=10, width=200)

        self.high_thresh_scale = tk.Scale(self.root, from_=0, to=255, orient=tk.HORIZONTAL, label="High Canny Threshold")
        self.high_thresh_scale.set(self.high_threshold)
        self.high_thresh_scale.place(x=10, y=70, width=200)

        self.min_area_scale = tk.Scale(self.root, from_=100, to=10000, orient=tk.HORIZONTAL, label="Minimum Area")
        self.min_area_scale.set(self.min_area)
        self.min_area_scale.place(x=10, y=130, width=200)

        self.video_label = tk.Label(self.root)
        self.video_label.place(x=220, y=10, width=320, height=240)  # Adjusted size

        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.root.after(100, self.update_parameters)
        self.root.after(100, self.update_video_feed)
        self.root.mainloop()

    def update_parameters(self):
        self.low_threshold = self.low_thresh_scale.get()
        self.high_threshold = self.high_thresh_scale.get()
        self.min_area = self.min_area_scale.get()
        self.root.after(100, self.update_parameters)

    def update_video_feed(self):
        if self.cv_image is not None:
            # Resize image to half size
            small_image = cv2.resize(self.cv_image, (0, 0), fx=0.5, fy=0.5)
            cv_image_rgb = cv2.cvtColor(small_image, cv2.COLOR_BGR2RGB)
            pil_image = PILImage.fromarray(cv_image_rgb)
            imgtk = ImageTk.PhotoImage(image=pil_image)
            self.video_label.imgtk = imgtk
            self.video_label.configure(image=imgtk)
        self.root.after(100, self.update_video_feed)

    def image_callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, self.low_threshold, self.high_threshold)
        contours, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        self.detected = False
        for cnt in contours:
            epsilon = 0.02 * cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, epsilon, True)
            if len(approx) == 4:
                area = cv2.contourArea(approx)
                if area > self.min_area:
                    self.detected = True
                    self.cv_image = cv2.drawContours(self.cv_image, [approx], -1, (0, 255, 0), 3)

        status = Int16()
        if self.detected:
            rospy.loginfo("Box detected")
            status.data = 1
        else:
            rospy.loginfo("No box detected")
            status.data = 0

        self.pub.publish(status)

        # Introduce a delay
        time.sleep(0.01)  # Sleep for 500ms to reduce the callback frequency

    def on_closing(self):
        rospy.signal_shutdown("GUI closed")
        self.root.destroy()

    def check_ros(self):
        if not rospy.is_shutdown():
            self.root.after(100, self.check_ros)
        else:
            self.root.destroy()

def main():
    rospy.init_node('box_detector', anonymous=True)
    BoxDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

