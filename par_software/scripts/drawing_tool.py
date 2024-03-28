#!/usr/bin/env python

import rospy
import numpy as np

from interface.par_interface import PixArtInterface

import tkinter as tk

class DrawingTool():
    def __init__(self, robot, on_off_dict):
        self.robot = robot

        self.width = self.robot.display_W * self.robot.module_size
        self.height = self.robot.display_H * self.robot.module_size

        self.pixel_size = 20  # Adjust this value to change the size of each pixel
        self.canvas_width = self.width * self.pixel_size
        self.canvas_height = self.height * self.pixel_size

        self.pixel_off_value = on_off_dict["off"]
        self.pixel_on_value = on_off_dict["on"]

        self.root = tk.Tk()
        self.root.title("Drawing Tool")

        self.canvas = tk.Canvas(self.root, width=self.canvas_width, height=self.canvas_height, bg="white")
        self.canvas.pack()

        self.canvas.bind("<Button-1>", self.draw_pixel)

        self.clear_button = tk.Button(self.root, text="Clear", command=self.clear_canvas)
        self.clear_button.pack()

        self.publish_button = tk.Button(self.root, text="Publish", command=self.publish_image)
        self.publish_button.pack()

        self.pixel_array = [[0 for _ in range(self.width)] for _ in range(self.height)]

    def draw_pixel(self, event):
        x = event.x // self.pixel_size
        y = event.y // self.pixel_size
        if 0 <= x < self.width and 0 <= y < self.height:
            if self.pixel_array[y][x] == self.pixel_off_value:
                self.pixel_array[y][x] = self.pixel_on_value
                self.canvas.create_rectangle(x * self.pixel_size, y * self.pixel_size,
                                            (x + 1) * self.pixel_size, (y + 1) * self.pixel_size,
                                            fill="black", outline="black")
            else:
                self.pixel_array[y][x] = self.pixel_off_value
                self.canvas.create_rectangle(x * self.pixel_size, y * self.pixel_size,
                                            (x + 1) * self.pixel_size, (y + 1) * self.pixel_size,
                                            fill="white", outline="white")
                # Redraw the border of the rectangle representing the erased pixel
                self.canvas.create_rectangle(x * self.pixel_size, y * self.pixel_size,
                                            (x + 1) * self.pixel_size, (y + 1) * self.pixel_size,
                                            outline="white")

    def clear_canvas(self):
        self.canvas.delete("all")
        self.pixel_array = [[0 for _ in range(self.width)] for _ in range(self.height)]

    def publish_image(self):
        self.robot.publish_to_modules(np.asarray(self.pixel_array))
        print("Publishing image:")
        for row in self.pixel_array:
            print(row)

    def run(self):
        self.root.mainloop()

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

        on_off_dict = {"on": 30, "off": 0}
        drawing_tool = DrawingTool(robot = par_interface, on_off_dict = on_off_dict)
        drawing_tool.run()

    except rospy.ROSInterruptException:
        pass
