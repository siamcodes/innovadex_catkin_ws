#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int16
import tkinter as tk
from PIL import Image as PILImage, ImageTk

class CircleDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        self.pub = rospy.Publisher("circle_detect_status", Int16, queue_size=10)
        self.detected = False

        self.canny_threshold = 200
        self.accumulator_threshold = 50
        self.gaussian_blur_size = 9
        self.dp = 1.2
        self.min_dist_between_circles = 20
        self.min_radius = 0
        self.max_radius = 0

        self.cv_image = None

        self.setup_gui()

    def setup_gui(self):
        self.root = tk.Tk()
        self.root.title("Circle Detector Parameters and Video Feed")
        self.root.geometry("600x400")

        self.canvas = tk.Canvas(self.root, width=600, height=400)
        self.canvas.pack()

        self.canny_thresh_scale = tk.Scale(self.root, from_=1, to=300, orient=tk.HORIZONTAL, label="Canny Threshold")
        self.canny_thresh_scale.set(self.canny_threshold)
        self.canny_thresh_scale.place(x=10, y=10, width=200)

        self.accum_thresh_scale = tk.Scale(self.root, from_=1, to=300, orient=tk.HORIZONTAL, label="Accumulator Threshold")
        self.accum_thresh_scale.set(self.accumulator_threshold)
        self.accum_thresh_scale.place(x=10, y=70, width=200)

        self.blur_size_scale = tk.Scale(self.root, from_=1, to=30, orient=tk.HORIZONTAL, label="Gaussian Blur Size")
        self.blur_size_scale.set(self.gaussian_blur_size)
        self.blur_size_scale.place(x=10, y=130, width=200)

        self.min_radius_scale = tk.Scale(self.root, from_=0, to=100, orient=tk.HORIZONTAL, label="Minimum Radius")
        self.min_radius_scale.set(self.min_radius)
        self.min_radius_scale.place(x=10, y=190, width=200)

        self.max_radius_scale = tk.Scale(self.root, from_=0, to=200, orient=tk.HORIZONTAL, label="Maximum Radius")
        self.max_radius_scale.set(self.max_radius)
        self.max_radius_scale.place(x=10, y=250, width=200)

        self.video_label = tk.Label(self.root)
        self.video_label.place(x=220, y=10, width=360, height=270)

        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.root.after(100, self.update_parameters)
        self.root.after(100, self.update_video_feed)
        self.root.mainloop()

    def update_parameters(self):
        self.canny_threshold = self.canny_thresh_scale.get()
        self.accumulator_threshold = self.accum_thresh_scale.get()
        self.gaussian_blur_size = self.blur_size_scale.get()
        # Ensure the blur size is an odd number
        if self.gaussian_blur_size % 2 == 0:
            self.gaussian_blur_size += 1
        self.min_radius = self.min_radius_scale.get()
        self.max_radius = self.max_radius_scale.get()
        self.root.after(100, self.update_parameters)

    def update_video_feed(self):
        if self.cv_image is not None:
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
        blurred = cv2.GaussianBlur(gray, (self.gaussian_blur_size, self.gaussian_blur_size), 0)
        circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, dp=self.dp, minDist=self.min_dist_between_circles,
                                   param1=self.canny_threshold, param2=self.accumulator_threshold,
                                   minRadius=self.min_radius, maxRadius=self.max_radius)

        self.detected = False
        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            for (x, y, r) in circles:
                cv2.circle(self.cv_image, (x, y), r, (0, 255, 0), 4)
                cv2.rectangle(self.cv_image, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
                self.detected = True

        status = Int16()
        if self.detected:
            rospy.loginfo("Circle detected")
            status.data = 1
        else:
            rospy.loginfo("No circle detected")
            status.data = 0
        self.pub.publish(status)

    def on_closing(self):
        rospy.signal_shutdown("GUI closed")
        self.root.destroy()

    def check_ros(self):
        if not rospy.is_shutdown():
            self.root.after(100, self.check_ros)
        else:
            self.root.destroy()

def main():
    rospy.init_node('circle_detector', anonymous=True)
    CircleDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

