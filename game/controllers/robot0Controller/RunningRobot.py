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
            # navigate(self)
            update_dir(self)
            move_forward2(self, 6)
            # set_left_vel(self, 0)
            # set_right_vel(self, 0)

            run_simulation(self, 16)




