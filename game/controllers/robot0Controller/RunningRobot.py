from functions import *
from MazeRobot import MazeRobot
import cv2
import numpy as np

TIME_STEP = 32


class RunningRobot(MazeRobot):
    def run(self):

        while self.robot.step(TIME_STEP) != -1:
            # print("x = ", x)
            get_all_values(self)
            # print(self.gyro_values, "7aga 7elwa........")
            if self.time_step >= 50:
                if move_one_tile(self):
                    add_to_arr(self.tile_pos, self.robot_pos.copy())
            print(self.robot_position_list)
            print(self.tile_pos, "Tile_pos", end="\n_____________\n")
            if len(self.tile_pos) >= 3:
                print("1st: {}, 2nd {}".format(get_dist(self.tile_pos[0:1]), get_dist(self.tile_pos[1:2])))

            """
            # get_wall(self)
            # print("Wall:", self.lidar_wall, end="\n______")
            if self.time_step > 50:
                # turn_right(self, 6.28)
                turn_90_time_step(self)
            """
            if self.time_step > 50:
                move_forward(self, 6.28)
                print()
            else:
                stop(self)
