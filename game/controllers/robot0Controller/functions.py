from MazeRobot import MazeRobot

def set_left_vel(robot: MazeRobot, v):
    robot.left_wheel.setVelocity(-v)

def set_right_vel(robot: MazeRobot, v):
    robot.right_wheel.setVelocity(-v)


def get_gps(robot: MazeRobot):
    robot.robot_pos[0] = robot.gps.getValues()[0]
    robot.robot_pos[1] = robot.gps.getValues()[2]

def get_all_values(robot: MazeRobot):
    get_gps(robot)

def get_color_sensor(robot: MazeRobot):
    pass


