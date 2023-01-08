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

        start_server(self)

        while self.can_run_simulation:
            if self.current_status == "forward":
                set_left_vel(self, -6)
                set_right_vel(self, -6)
            elif self.current_status == "left":
                # set_left_vel(self, 2)
                # set_right_vel(self, -2)
                set_left_vel(self, 2.2)
                set_right_vel(self, -2.2)
            elif self.current_status == "right":
                # set_left_vel(self, -2)
                # set_right_vel(self, 2)


                set_left_vel(self, -2.2)
                set_right_vel(self, 2.2)

            elif self.current_status == "backward":
                set_left_vel(self, 6)
                set_right_vel(self, 6)
            elif self.current_status == "stop":
                set_left_vel(self, 0)
                set_right_vel(self, 0)
            elif self.current_status == "right90":
                turn_90_time_step(self, "right")
                self.current_status = "stop"
                set_left_vel(self, 0)
                set_right_vel(self, 0)

            elif self.current_status == "left90":
                turn_90_time_step(self, "left")
                self.current_status = "stop"
                set_left_vel(self, 0)
                set_right_vel(self, 0)

            run_simulation(self)

