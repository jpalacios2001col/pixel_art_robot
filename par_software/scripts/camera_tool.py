#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from interface.par_interface import PixArtInterface

class CameraTool:
    def __init__(self, robot):
        self.robot = robot

        self.width = self.robot.display_W * self.robot.module_size
        self.height = self.robot.display_H * self.robot.module_size

        # Initialize webcam
        self.cap = cv2.VideoCapture(0)  # Change index if you have multiple cameras
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

    def process_frame(self):
        ret, frame = self.cap.read()
        if ret:
            print("Original frame dimensions:", frame.shape)

            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            pixel_frame = self.pixelize(gray_frame, self.robot.module_size)

            print("Pixel frame dimensions:", pixel_frame.shape)

            self.robot.publish_to_modules(pixel_frame)

    def pixelize(self, frame, module_size):
        h, w = frame.shape[:2]

        # Calculate new dimensions that are multiples of module_size
        new_h = (h // module_size) * module_size
        new_w = (w // module_size) * module_size

        # Resize frame to ensure dimensions are multiples of module_size
        frame = cv2.resize(frame, (new_w, new_h))

        # Pixelate the resized frame
        mh, mw = new_h // module_size, new_w // module_size
        pixel_frame = np.zeros((mh, mw), dtype=np.uint8)
        for i in range(mh):
            for j in range(mw):
                pixel_frame[i, j] = np.mean(frame[i * module_size: (i + 1) * module_size,
                                                j * module_size: (j + 1) * module_size])
        
        # Resize pixel_frame:
        pixel_frame = cv2.resize(pixel_frame, (self.width, self.height))

        return pixel_frame

    def run(self):
        rate = rospy.Rate(100)  # Adjust the frequency as needed
        while not rospy.is_shutdown():
            self.process_frame()
            rate.sleep()

if __name__ == '__main__':
    try:
        # Initialize the ROS node
        rospy.init_node('camera_tool', anonymous=True)

        # Create a PixArtInterface instance
        par_interface = PixArtInterface(module_size=4, display_shape=[3, 3])

        # Create a CameraTool instance
        camera_tool = CameraTool(robot=par_interface)

        # Run the camera tool
        camera_tool.run()

    except rospy.ROSInterruptException:
        pass
