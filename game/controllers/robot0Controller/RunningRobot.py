from functions import *
from Camera import *
from MazeRobot import MazeRobot
import cv2
import time
import numpy as np
from server import server, start_server

TIME_STEP = 32


class RunningRobot(MazeRobot):
    def run(self):

        set_left_vel(self, 0)
        set_right_vel(self, 0)
        stop(self, 150)
        # start_server(self)
        while self.can_run_simulation:
            # move_one_tile(self)
            # print(self.current_status)
            # print(self.lidar_data[2][128])

            while self.lidar_data[2][0] > 11.54:
                move_forward2(self, -6)
                for i in range(0, 256, 16):
                    print(self.lidar_data[2][i], i)
                print("------------------")
                # print("f {}   b {}   l {}   r {}".format(self.lidar_data[2][0], self.lidar_data[2][256], self.lidar_data[2][384], self.lidar_data[2][128]))
                run_simulation(self, 16)

            turn_90_time_step(self, "right")
            # print(self.lidar_data[2][0])

            run_simulation(self, 16)



