# initialize a robot on the middle of a a square grid map
# the robot is either able to rotate or move forward
# the robot position is described by x, y and theta
# the robot is able to sense if it has reached a wall

import cv2
import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation
import random

Width = 400
Height = 400


class Robot:
    def __init__(self, x0, y0, t0):
        self.x = x0
        self.y = y0
        self.t = t0
        self.radius = 10
        self.velocity = 3
        self.angular_velocity = 0.1
        self.to_rotate = False
        self.map = [Height, Width]
        self.canvas = np.zeros((self.map[0], self.map[1], 3), np.uint8)

    def show(self):
        # visualize robot position with circle and arrow on the map
        plt.clf()
        circle = plt.Circle((self.x, self.y), radius=self.radius, color='red', fill=False)
        plt.gca().add_patch(circle)
        # Draw the arrowed line
        arrow_len = 10
        arrow_dx = arrow_len * np.cos(self.t)
        arrow_dy = arrow_len * np.sin(self.t)
        arrow = plt.Arrow(self.x, self.y, arrow_dx, arrow_dy, width=0.5, color='green')
        plt.gca().add_patch(arrow)
        return plt.imshow(self.canvas)

    def update(self, i):
        print(i)
        self.check_collision()
        self.translate()
        return self.show()

    def check_collision(self):
        angle = self.t
        if self.x <= self.radius:
            angle = np.random.uniform(-np.pi / 2, np.pi / 2)
        if self.x >= self.map[1] - self.radius:
            angle = np.random.uniform(np.pi / 2, 3 * np.pi / 2)
        if self.y <= self.radius:
            angle = np.random.uniform(0, np.pi)
        if self.y >= self.map[0] - self.radius:
            angle = np.random.uniform(-np.pi, 0)
        self.rotate(angle)

    def rotate(self, angle):
        self.t = angle


    def translate(self):
        self.x += self.velocity * np.cos(self.t)
        self.y += self.velocity * np.sin(self.t)


robot = Robot(Height / 2, Width / 2, 0)
ani = animation.FuncAnimation(plt.gcf(), robot.update, frames=1000)
ani.save('robot_simulation.mp4', writer='ffmpeg', fps=30)
plt.show()
