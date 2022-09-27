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

            image = self.color_sensor.getImage()  # Step 4: Retrieve the image frame.

            # Get the individual RGB color channels from the pixel (0,0)
            # Note that these functions require you to pass the width of the overall image in pixels.
            # Since this is a 1px by 1px color sensor, the width of the image is just 1.
            r = self.color_sensor.imageGetRed(image, 1, 0, 0)
            g = self.color_sensor.imageGetGreen(image, 1, 0, 0)
            b = self.color_sensor.imageGetBlue(image, 1, 0, 0)

            print("r: " + str(r) + " g: " + str(g) + " b: " + str(b))
