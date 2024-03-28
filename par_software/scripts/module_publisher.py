#!/usr/bin/env python3

import rospy
from par_msgs.msg import module_msg, pixel_msg

def publisher():
    # Initialize the ROS node
    rospy.init_node('module_publisher', anonymous=True)

    # Create a publisher for the /module_topic topic
    module_pub = rospy.Publisher('/module_topic', module_msg, queue_size=10)

    # Define the rate at which to publish messages (in Hz)
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        # Create a new module message
        module_message = module_msg()

        # Populate the existing pixels array with servo angles (for demonstration purposes)
        for i in range(16):
            module_message.pixels[i].angle = i * 10  # Example servo angle

        print(module_message)

        # Publish the module message
        module_pub.publish(module_message)

        # Log message for debugging (optional)
        rospy.loginfo("Published module message")

        # Sleep to maintain the specified publishing rate
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
