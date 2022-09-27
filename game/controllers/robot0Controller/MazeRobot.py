from functions import *

TIME_STEP = 32

class MazeRobot:
    def __init__(self, robot):
        self.robot = robot
        self.left_wheel = robot.getDevice("wheel1 motor")  # Motor initialization
        self.right_wheel = robot.getDevice("wheel2 motor")
        self.left_wheel.setPosition(float('inf'))
        self.right_wheel.setPosition(float('inf'))
        self.left_camera = robot.getDevice("camera_l")
        self.right_camera = robot.getDevice("camera_r")
        self.left_camera.enable(TIME_STEP)
        self.right_camera.enable(TIME_STEP)

    def set_left_vel(self, v):
        self.left_wheel.setVelocity(v)

    def set_right_vel(self, v):
        self.right_wheel.setVelocity(v)


    def run(self):
        while self.robot.step(TIME_STEP) != -1:
            self.set_right_vel(0)
            self.set_left_vel(0)
