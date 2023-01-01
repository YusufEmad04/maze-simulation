from functions import *
from Camera import *
from MazeRobot import MazeRobot
import cv2
import time
import numpy as np

TIME_STEP = 32


class RunningRobot(MazeRobot):
    def run(self):
        x = 0
        # while self.robot.step(TIME_STEP) != -1:
        #     get_all_values(self)
        #     if x % 10 == 0:
        #         turn_90_time_step(self)
        #     else:
        #         stop(self)
        #
        #     x += 1
        #     print(self.current_direction)
        run_simulation(self)
        while self.can_run_simulation:

            if x % 10 == 0:
                turn_90_time_step(self)
            else:
                stop(self)

            x += 1
            print(self.current_direction)

            run_simulation(self)