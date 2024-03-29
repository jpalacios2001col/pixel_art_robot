#!/usr/bin/env python

import rospy
import numpy as np
from par_msgs.msg import module_msg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class PixelArtRobot:
    def __init__(self, module_size=4, display_shape=[2, 2], frequency=5):
        self.module_size = int(module_size)
        self.display_shape = np.asarray(display_shape)
        self.display_H = self.display_shape[0]
        self.display_W = self.display_shape[1]
        self.num_modules = self.display_shape[0] * self.display_shape[1]
        self.image_shape = tuple(self.display_shape * module_size)

        self.scale_factor = 100

        self.current_image = np.zeros(self.image_shape, dtype=np.uint8)

        self.frequency = frequency
        self.bridge = CvBridge()

        self.subscribers = []
        for i in range(self.num_modules):
            topic = '/module_topic_' + str(i)
            sub = rospy.Subscriber(topic, module_msg, self.update_image, callback_args=i)
            self.subscribers.append(sub)

        self.timer = rospy.Timer(rospy.Duration(1.0 / self.frequency), self.render_image)

    def update_image(self, msg, module_num):
        pixel_array = np.array(msg.pixels).reshape((self.module_size, self.module_size))
        start_i = (module_num // self.display_W) * self.module_size
        start_j = (module_num % self.display_W) * self.module_size
        self.current_image[start_i:start_i + self.module_size, start_j:start_j + self.module_size] = pixel_array

    def render_image(self, event):
        # Scale up the image
        scaled_image = cv2.resize(self.current_image, None, fx=self.scale_factor, fy=self.scale_factor, interpolation=cv2.INTER_NEAREST)
        cv2.imshow('Current Image', scaled_image)
        cv2.waitKey(1)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('pixel_art_robot')
    robot = PixelArtRobot(module_size=4, display_shape=[2, 3], frequency=100)
    robot.run()
