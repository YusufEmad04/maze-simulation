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

        self.gyro = robot.getDevice("gyro")
        self.gyro.enable(TIME_STEP)


        self.robot_pos = [0, 0]
        self.color_sensor_values = [0, 0, 0]

        self.image = None

        self.color_case=""
        self.start_point=0 #detecting the start point

        self.stoptime=0
        self.found_victim=0
        self.victim_status='N'
        self.victim_timer=0
              
        self.left_encoder = self.left_wheel.getPositionSensor()    # Encoder initialization
        self.right_encoder = self.right_wheel.getPositionSensor()
        self.left_encoder.enable(TIME_STEP)
        self.right_encoder.enable(TIME_STEP)

    
        self.emitter = robot.getDevice("emitter")
        self.x_dimension=-54
        self.z_dimension=-54
        self.tiles_cnt = int((abs(self.x_dimension) + abs(self.z_dimension))/12)
        self.map =[[-1 for k in range(self.tiles_cnt*4-3)] for j in range(self.tiles_cnt*4-3)]
