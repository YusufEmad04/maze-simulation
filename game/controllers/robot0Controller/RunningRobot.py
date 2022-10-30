from functions import *
from MazeRobot import MazeRobot
import cv2
import numpy as np

TIME_STEP = 32


class RunningRobot(MazeRobot):
    def run(self):

        x = 0

        while self.robot.step(TIME_STEP) != -1:
            x += 2
            print("x = ", x)
            get_all_values(self)
            # print gyro values
            print("gyro values: ", self.gyro_values)
            if x % 150 == 0:
                # turn_90_time_step(self)
                turn_90_time_step(self)
            else:
                # stop
                set_left_vel(self, 0)
                set_right_vel(self, 0)
