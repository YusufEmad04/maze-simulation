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
        run_simulation(self)
        while self.can_run_simulation:

            x += 1
            if x % 10 == 0:
                turn_90_time_step(self)
            else:
                stop(self)
            # if self.current_direction != 1:
            #     turn_90_time_step(self)
            # else:
            #     # print("Abs_Pos: {}\nGPS:: {}\n___________".format(self.abs_pos, self.robot_pos))
            #     move_one_tile_gps(self)
            #
            #     # print(self.current_direction)

            run_simulation(self)
