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
        while self.robot.step(TIME_STEP) != -1:
            get_all_values(self)
            x += 2
            if x % 20 == 0:
                turn_90_time_step(self)
            else:
                stop(self)
            # stop(self)
            # # print_dict(check_walls(self))
            # half_wall_index = int(math.atan2(1, 2) * 512 / (2 * math.pi))
            # print(half_wall_index)
            # print(self.lidar_data[2][half_wall_index])
        # while self.robot.step(TIME_STEP) != -1:
        #     if not (self.current_direction == 1) and self.time_step < 3:
        #         turn_90_time_step_with_camera(self, "right")
        #     else:
        #
        #         get_all_values(self)
        #         move_one_tile_gps(self)
