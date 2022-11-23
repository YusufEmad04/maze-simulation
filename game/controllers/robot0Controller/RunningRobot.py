from functions import *
from MazeRobot import MazeRobot
import cv2
import numpy as np

TIME_STEP = 32


class RunningRobot(MazeRobot):
    def run(self):

        while self.robot.step(TIME_STEP) != -1:
            get_all_values(self)

            if not (self.current_direction == 2):
                turn_90_time_step(self, "right")
            if self.time_step >= 50:
                if move_one_tile_gps(self):
                    add_to_arr(self.tile_pos, self.robot_pos.copy())

            # stop(self)
            # self.wanted_tile = (self.start_tile[0] + 12, self.start_tile[1])
            if arrived_at_coord(self, self.wanted_tile):
                print("ARRIVEDDDDD!!!!!!!!!!!!!!!!!! \n_______________")

            print(self.wanted_tile, "IS WANTED")
            # print("Distance away:", get_dist(self.wanted_tile, self.robot_pos))
            print(self.robot_position_list)
            print(self.tile_pos, "Tile_pos", end="\n_____________\n")
            if len(self.tile_pos) >= 3:
                print("1st: {}, 2nd {}".format(get_dist(self.tile_pos[0], self.tile_pos[1]),
                                               get_dist(self.tile_pos[1], self.tile_pos[2])))
