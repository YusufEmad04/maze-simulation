TIME_STEP = 32


class MazeRobot:
    def __init__(self, robot):
        self.robot = robot

        self.time_step = 0

        self.can_run_simulation = True

        self.right_wheel = robot.getDevice("wheel1 motor")
        self.right_wheel.setPosition(float('inf'))

        self.left_wheel = robot.getDevice("wheel2 motor")
        self.left_wheel.setPosition(float('inf'))

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
        self.lidar.enablePointCloud()

        self.gyro = robot.getDevice("gyro")
        self.gyro.enable(TIME_STEP)

        self.robot_pos = [0, 0]
        self.color_sensor_values = [0, 0, 0]
        self.gyro_values = [0, 0, 0]
        self.lidar_data = []
        self.lidar_data_point_cloud = []
        self.lidar_groups = [-1] * 12
        self.lidar_wall = [0, 0, 0, 0]

        self.left_image = None
        self.right_image = None

        self.color_case = ""

        self.directions = ["front", "right", "back", "left"]
        self.current_direction = 0

        self.left_encoder = self.left_wheel.getPositionSensor()  # Encoder initialization
        self.right_encoder = self.right_wheel.getPositionSensor()
        self.left_encoder.enable(TIME_STEP)
        self.right_encoder.enable(TIME_STEP)

        self.robot_position_list = []
        self.time_steps_arr = []

        self.tile_pos = []
        self.abs_pos = [-1, -1]                 # Position where robot supposed to be
        self.current_tile = [-1, -1]
        self.wanted_tile = [-1, -1]

        self.emitter = robot.getDevice("emitter")

        self.maze_x_size = 0
        self.maze_y_size = 0
        # self.tiles_cnt = int((abs(self.x_dimension) + abs(self.z_dimension)) / 12)
        self.map = [
            ["0", "0", "0", "0", "0"],
            ["0", "0", "0", "0", "0"],
            ["0", "0", "0", "0", "0"],
            ["0", "0", "0", "0", "0"],
            ["0", "0", "0", "0", "0"]
        ]

        self.current_status = "stop"

