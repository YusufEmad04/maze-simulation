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

            # if self.current_status == "capture":
            #     self.current_status = "stop"
            #     # save images
            #     cv2.imwrite("images/left{}.jpg".format(self.counter), self.left_image)
            #     cv2.imwrite("images/right{}.jpg".format(self.counter), self.right_image)

            set_left_vel(self, 5)
            set_right_vel(self, -5)

            run_simulation(self)

