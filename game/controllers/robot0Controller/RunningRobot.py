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
        update_dir(self)
        # start_server(self)
        while self.can_run_simulation:
            # if self.strategy:
            #     navigate2(self)
            # else:
            #     move_bfs(self)
            # navigate2(self)
            if self.strategy == 0:
                navigate2(self)
            elif self.strategy == 1:
                move_bfs(self)

            elif self.strategy == 2:
                navigate3(self)

            # navigate3(self)

            run_simulation(self, 16)




