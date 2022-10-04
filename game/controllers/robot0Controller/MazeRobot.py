TIME_STEP = 32

class MazeRobot:
    def __init__(self, robot):
        self.robot = robot

        self.left_wheel = robot.getDevice("wheel1 motor")
        self.left_wheel.setPosition(float('inf'))

        self.right_wheel = robot.getDevice("wheel2 motor")
        self.right_wheel.setPosition(float('inf'))

        self.left_camera = robot.getDevice("camera_l")
        self.left_camera.enable(TIME_STEP)

        self.right_camera = robot.getDevice("camera_r")
        self.right_camera.enable(TIME_STEP)

        self.color_sensor = robot.getDevice("colour_sensor")
        self.color_sensor.enable(TIME_STEP)

        self.gps = robot.getDevice("gps")
        self.gps.enable(TIME_STEP)

        self.lidar = robot.getDevice("lidar")
        self.lidar.enable(TIME_STEP)

        self.robot_pos = [0, 0]
        self.color_sensor_values = [0, 0, 0]

        self.image = None


    def run(self):
        pass