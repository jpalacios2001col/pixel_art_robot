#!/usr/bin/env python

import rospy
import numpy as np

from interface.par_interface import PixArtInterface

if __name__ == '__main__':
    try:
        # Initialize the ROS node:
        rospy.init_node('par_commander', anonymous=True)

        # Create a PixArtInterface instance:
        par_interface = PixArtInterface(module_size = 4, display_shape = [2,2])

        '''
        Testing functionality:
        '''

        # Extract Variables:
        num_modules = par_interface.num_modules
        display_shape = par_interface.display_shape
        module_size = par_interface.module_size

        # Create an image to size:
        numbers = np.arange(1, num_modules * module_size**2 + 1)
        image = numbers.reshape((module_size * display_shape[0], module_size * display_shape[1]))

        print(image)

        # Set the rate to 5 Hz
        rate = rospy.Rate(5)

        # Run the publish_to_modules() method at 5 Hz
        while not rospy.is_shutdown():
            par_interface.publish_to_modules(image)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
