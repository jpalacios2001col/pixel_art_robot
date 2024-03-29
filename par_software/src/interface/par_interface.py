#!/usr/bin/env python

import rospy
from par_msgs.msg import module_msg
import numpy as np

class PixArtInterface(object):
    def __init__(self, module_size=4, display_shape=[2,2]):
        # Display dimension inputs:
        self.module_size = int(module_size)
        self.display_shape = np.asarray(display_shape)

        self.display_H = self.display_shape[0]
        self.display_W = self.display_shape[1]

        # Display dimension variables:
        self.num_modules = self.display_shape[0] * self.display_shape[1]
        self.image_shape = tuple(self.display_shape * module_size)

        # Module publisher topics:
        self.module_pubs = [rospy.Publisher('/module_topic_' + str(i), module_msg, queue_size=10) for i in range(self.num_modules)]

    def publish_to_modules(self, pixel_array, verbose=False):
        # Check input size:
        if pixel_array.shape != self.image_shape:
            rospy.logerr("Input pixel array size does not match the expected size")
            rospy.logerr(f"Input size: {pixel_array.shape}")
            rospy.logerr(f"Expected size: {self.image_shape}")
            return

        # Publish the pixel angles to the respective module topics
        module_num = 0

        for mod_i in range(self.display_H):         # Columns of display modules
            for mod_j in range(self.display_W):     # Rows of display modules
                
                # Index subset of pixel_array that corresponds to module:
                pixel_subset = pixel_array[mod_i * self.module_size: (mod_i+1) * self.module_size,
                                            mod_j * self.module_size: (mod_j+1) * self.module_size]
                pixel_values = pixel_subset.flatten().tolist()

                # Create and fill message:
                mod_msg = module_msg()
                mod_msg.header.stamp = rospy.Time.now()
                mod_msg.module_size = self.module_size
                mod_msg.pixels = pixel_values

                # Publish message:
                self.module_pubs[module_num].publish(mod_msg)

                rospy.loginfo(f"Pixel values: {pixel_values}")

                if verbose:
                    rospy.loginfo(f"Publishing pixel subset to module {module_num}:\n{pixel_subset}")
                    rospy.loginfo(f"Pixel values: {pixel_values}")
                    rospy.loginfo(f"Module message:\n{mod_msg}")
                    rospy.loginfo(f"Published module message to /module_topic_{module_num}")

                module_num += 1