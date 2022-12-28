from functions import *
from  Camera import *
from MazeRobot import MazeRobot
import cv2
import time
import numpy as np

TIME_STEP = 32


class RunningRobot(MazeRobot):
    

    def run(self):

        while self.robot.step(TIME_STEP) != -1:
            if not (self.current_direction == 1) and self.time_step < 3:
                turn_90_time_step_with_camera(self, "right")
            else:

                get_all_values(self)
                wall_dict = check_walls(self)
                front=wall_dict['front']
                # print(f"dict: {wall_dict}")
                if not wall_dict['right']:
                    turn_90_time_step_with_camera(self, "left")
                    move_one_tile(self)
                elif front:
                    turn_90_time_step_with_camera(self, "right")
                else:
                    move_one_tile(self)
            

            
            
            
            
            # else:
            
            # move_one_tile_gps_with_camera(self, self.left_image)
            # add_to_arr(self.tile_pos, self.robot_pos.copy())
            # get_all_values(self)
            # wall_dict = check_walls(self)
            # front=wall_dict['front']



            # if self.time_step >= 50:
            #     if move_one_tile_gps_with_camera(self, self.left_image):
            #         print('1 tile moved')
            #         stop(self)
            #         time.sleep(1)
            #         add_to_arr(self.tile_pos, self.robot_pos.copy())

            # stop(self)
            # # self.wanted_tile = (self.start_tile[0] + 12, self.start_tile[1])
            # if arrived_at_coord(self, self.wanted_tile):
            #     print("ARRIVEDDDDD!!!!!!!!!!!!!!!!!! \n_______________")

            # print(self.wanted_tile, "IS WANTED")
            # # print("Distance away:", get_dist(self.wanted_tile, self.robot_pos))
            # print(self.robot_position_list)
            # print(self.tile_pos, "Tile_pos", end="\n_____________\n")
            # if len(self.tile_pos) >= 3:
            #     print("1st: {}, 2nd {}".format(get_dist(self.tile_pos[0], self.tile_pos[1]),
            #                                    get_dist(self.tile_pos[1], self.tile_pos[2])))

            # camera_image_L = self.left_camera.getImage()
            # camera_image_L = np.frombuffer(camera_image_L, np.uint8).reshape((self.left_camera.getHeight(), self.left_camera.getWidth(), 4))
            # img_L = cv2.cvtColor(camera_image_L, cv2.COLOR_BGRA2BGR)
            # full_detection(img_L)
            
            # camera_image_R = self.right_camera.getImage()
            # camera_image_R = np.frombuffer(camera_image_R, np.uint8).reshape((self.right_camera.getHeight(), self.right_camera.getWidth(), 4))
            # img_R = cv2.cvtColor(camera_image_R, cv2.COLOR_BGRA2BGR)
            # cam(self,img_R)
