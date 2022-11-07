from functions import *
from MazeRobot import MazeRobot
import cv2
import numpy as np

TIME_STEP = 32


class RunningRobot(MazeRobot):
    def run(self):

        while self.robot.step(TIME_STEP) != -1:
            # print("x = ", x)
            get_all_values(self)
            print(self.gyro_values, "7aga 7elwa........")
            # print(self.time_step)
            # get_wall(self)
            # print("Wall:", self.lidar_wall, end="\n______")
            if self.time_step > 50:
                # turn_right(self, 6.28)
                turn_90_time_step(self)
            else:
                stop(self)
