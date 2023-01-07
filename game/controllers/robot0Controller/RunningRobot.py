from functions import *
from Camera import *
from MazeRobot import MazeRobot
import cv2
import time
import numpy as np
import threading
from server import server
import socket

TIME_STEP = 32


class RunningRobot(MazeRobot):
    def run(self):

        set_left_vel(self, 0)
        set_right_vel(self, 0)
        stop(self, 150)

        print("-----------------")
        print("write this ip address inside the app")
        print(str(socket.gethostbyname(socket.gethostname())) + ":5000")
        print("-----------------")

        threading.Thread(target=server, args=(self,)).start()


        while self.can_run_simulation:
            if self.current_status == "forward":
                set_left_vel(self, -6)
                set_right_vel(self, -6)
            elif self.current_status == "left":
                # set_left_vel(self, 2)
                # set_right_vel(self, -2)
                turn_90_time_step(self, "left")
                self.current_status = "stop"
                set_left_vel(self, 0)
                set_right_vel(self, 0)
            elif self.current_status == "right":
                # set_left_vel(self, -2)
                # set_right_vel(self, 2)

                turn_90_time_step(self, "right")
                self.current_status = "stop"
                set_left_vel(self, 0)
                set_right_vel(self, 0)

            elif self.current_status == "backward":
                set_left_vel(self, 6)
                set_right_vel(self, 6)
            elif self.current_status == "stop":
                set_left_vel(self, 0)
                set_right_vel(self, 0)

            run_simulation(self)

