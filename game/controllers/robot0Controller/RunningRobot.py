from functions import *
from MazeRobot import MazeRobot

TIME_STEP = 32

class RunningRobot(MazeRobot):
    def run(self):
        while self.robot.step(TIME_STEP) != -1:
            set_right_vel(self, 0)
            set_left_vel(self, 0)
            get_all_values(self)
            print(self.robot_pos)
            print(self.color_sensor_values)

