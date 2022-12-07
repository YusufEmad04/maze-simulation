import cv2
import math, statistics
import time
import numpy as np
from MazeRobot import MazeRobot
from Camera import moving_cam
from Camera import full_detection

max_speed = 6.28


def add_to_arr(arr, data):
    arr.append(data)
    if len(arr) >= 4:
        arr.pop(0)


def arrived_at_coord(robot: MazeRobot, coord):
    offset = 0.4
    if (coord[0] - offset <= robot.robot_pos[0] <= coord[0] + offset) and (
            coord[1] - offset <= robot.robot_pos[1] <= coord[1] + offset):
        return True
    
    return False


def set_left_vel(robot: MazeRobot, v):
    robot.left_wheel.setVelocity(-v)


def set_right_vel(robot: MazeRobot, v):
    robot.right_wheel.setVelocity(-v)


def move_forward(robot: MazeRobot, v):
    set_left_vel(robot, v)
    set_right_vel(robot, v)


def turn_right(robot: MazeRobot, v):
    set_left_vel(robot, v)
    set_right_vel(robot, -v)


def turn_left(robot: MazeRobot, v):
    set_left_vel(robot, -v)
    set_right_vel(robot, v)


def turn_90_time_step(robot: MazeRobot, direction):
    x = 18
    if robot.color_case == "orange":
        speed = 5.4464
    else:
        speed = 3.4868

    if direction == "right":
        speed = -speed
        robot.current_direction = (robot.current_direction + 1) % 4
    elif direction == "left":
        robot.current_direction = (robot.current_direction - 1) % 4

    while robot.robot.step(32) != -1 and x >= 0:
        get_all_values(robot)
        print("gyro values: ", robot.gyro_values)
        x -= 1
        turn_left(robot, speed)

    # TEMP Add a break after turn
    while robot.robot.step(32) != -1 and x >= -5:
        stop(robot)
        x -= 1


def move_one_tile(robot: MazeRobot):
    x = 28
    while robot.robot.step(32) != -1 and x >= 0:
        x -= 1
        move_forward(robot, 6.221)
    stop(robot)
    while x >= -7:
        stop(robot)
        x -= 1

    return True


def move_one_tile_gps(robot: MazeRobot):

    if robot.current_direction in [0, 1]:
        sign = -1
    else:
        sign = 1

    if robot.current_direction in [0, 2]:
        robot.wanted_tile = [robot.robot_pos[0] + sign * 12, robot.robot_pos[1]]
    else:
        robot.wanted_tile = [robot.robot_pos[0], robot.robot_pos[1] + sign * 12]

    print(robot.robot_pos)
    print(robot.wanted_tile)

    while robot.robot.step(32) != -1 and not arrived_at_coord(robot, robot.wanted_tile):
        get_all_values(robot)
        print("Distance away:", get_dist(robot.wanted_tile, robot.robot_pos))
        move_forward(robot, 6.221)
        print("----------")
    print("_____WP_______\n\n\n_________WP________")
    for i in range(50):
        stop(robot)
    print("_________Hoi__________")
    return True


def move_one_tile_gps_with_camera(robot: MazeRobot, img):
    if robot.current_direction in [0, 1]:
        sign = -1
    else:
        sign = 1
    if robot.current_direction in [0, 2]:
        robot.wanted_tile = [robot.robot_pos[0] + sign * 12, robot.robot_pos[1]]
    else:
        robot.wanted_tile = [robot.robot_pos[0], robot.robot_pos[1] + sign * 12]

    detected = False
    while robot.robot.step(32) != -1 and not arrived_at_coord(robot, robot.wanted_tile):

        get_all_values(robot)
        if moving_cam(robot.left_image):
            victim_type = full_detection(robot.left_image)
            print(f"detected {detected}")
            if victim_type!="N" and detected == False:
                detected = True
                print(f"Victim type = {victim_type}") # send to the receiver
                stop(robot)
                time.sleep(0.7)
        print("Distance away:", get_dist(robot.wanted_tile, robot.robot_pos))
        move_forward(robot, 3)
        print("----------")
    
    stop(robot)
    time.sleep(2)

    print("_____WP_______\n\n\n_________WP________")
    for i in range(50):
        stop(robot)
    print("_________Hoi__________")
    return True


def stop(robot: MazeRobot):
    set_left_vel(robot, 0)
    set_right_vel(robot, 0)


def get_gps(robot: MazeRobot):
    robot.robot_pos[0] = robot.gps.getValues()[0] * 100
    robot.robot_pos[1] = robot.gps.getValues()[2] * 100
    if robot.start_tile == [-1, -1]:
        print("Changing start")
        robot.start_tile = robot.robot_pos.copy()


def get_dist(pos1, pos2):
    y = (pos2[1] - pos1[1]) ** 2
    x = (pos2[0] - pos1[0]) ** 2
    return math.sqrt(y + x)


def get_speed(t, pos):
    dist = get_dist(pos[0], pos[-1])
    t = t[-1] - t[0]
    if t == 0:
        return 0
    return 100 * dist / t


def get_color_sensor(robot: MazeRobot):
    robot.image = robot.color_sensor.getImage()
    robot.color_sensor_values[0] = robot.color_sensor.imageGetRed(robot.image, 1, 0, 0)
    robot.color_sensor_values[1] = robot.color_sensor.imageGetGreen(robot.image, 1, 0, 0)
    robot.color_sensor_values[2] = robot.color_sensor.imageGetBlue(robot.image, 1, 0, 0)


def get_gyro_values(robot: MazeRobot):
    robot.gyro_values[0] = robot.gyro.getValues()[0]
    robot.gyro_values[1] = robot.gyro.getValues()[1]
    robot.gyro_values[2] = robot.gyro.getValues()[2]


def get_cameras_values(robot: MazeRobot):
    robot.left_image = np.frombuffer(robot.left_camera.getImage(), np.uint8).reshape(
        (robot.left_camera.getHeight(), robot.left_camera.getWidth(), 4))
    robot.left_image = cv2.cvtColor(robot.left_image , cv2.COLOR_BGRA2BGR)
    robot.right_image = np.frombuffer(robot.right_camera.getImage(), np.uint8).reshape(
        (robot.right_camera.getHeight(), robot.right_camera.getWidth(), 4))
    robot.right_image = cv2.cvtColor(robot.right_image , cv2.COLOR_BGRA2BGR)

    color_sensor_image = robot.color_sensor.getImage()
    robot.color_sensor_values[0] = robot.color_sensor.imageGetRed(color_sensor_image, 1, 0, 0)
    robot.color_sensor_values[1] = robot.color_sensor.imageGetGreen(color_sensor_image, 1, 0, 0)
    robot.color_sensor_values[2] = robot.color_sensor.imageGetBlue(color_sensor_image, 1, 0, 0)


def get_lidar(robot: MazeRobot):
    # Loop on lidar data and add it to a 2D array
    range_image = robot.lidar.getRangeImage()
    for layer in range(4):
        robot.lidar_data.append([])
        for point in range(512):
            robot.lidar_data[layer].append(range_image[layer * 512 + point] * 100)
    lidar_group_values(robot)


def lidar_group_values(robot: MazeRobot):
    # Divide lidar into 12 groups (30 degrees each group) and take average value
    # Add values in a group to temp arr, get standard deviation, remnove outliers and calc mean
    for group in range(12):
        temp_arr = []
        # If front, get field of view (30 degree) in the front
        if group == 0:
            temp_arr.extend(robot.lidar_data[2][-20:])
            temp_arr.extend(robot.lidar_data[2][:19])
        else:
            start_index = (group * 43) - 19
            temp_arr.extend(robot.lidar_data[2][start_index: start_index + 43])

        # eliminate outliers and get mean
        mean = statistics.mean(temp_arr)
        stdev = statistics.stdev(temp_arr)

        final_arr = [x for x in temp_arr if (mean + 2 * stdev > x > mean - 2 * stdev)]
        # print("\nFinal ARRAY IS:\n", final_arr)
        robot.lidar_groups[group] = statistics.mean(final_arr)

    print("Groups:", robot.lidar_groups)


def get_wall(robot: MazeRobot):
    min_val = 4
    max_val = 8
    arr = [0, 0, 0, 0]
    print((robot.lidar_groups[8] + robot.lidar_groups[9]) / 2)
    if min_val <= robot.lidar_groups[0] <= max_val:
        arr[0] = 1
    if min_val <= (robot.lidar_groups[2] + robot.lidar_groups[3]) / 2 <= max_val:
        arr[1] = 1
    if min_val <= robot.lidar_groups[5] <= max_val:
        arr[2] = 1
    if min_val <= (robot.lidar_groups[8] + robot.lidar_groups[9]) / 2 <= max_val:
        arr[3] = 1
    robot.lidar_wall = arr


def create_tile(robot: MazeRobot):
    """
    Walls                                    1
    Holes                                    2
    Swamps                                   3
    Checkpoints                              4
    Starting tile                            5
    Connection tiles from area 1 to 2        6
    Connection tiles from area 1 to 3        7
    Connection tiles from area 2 to 3        8
    Victims    The corresponding victim code   (H,S,U,F,P,C,O)
    Any other tiles/edges/vertices           0
    """
    tile = [[0, 0, 0, 0] * 4]


# TODO We are creating double walls
def increase_maze_size(robot: MazeRobot, x, y, character):
    # loop on each row and add new character 5 times per new tile
    for i in robot.map:
        i.extend([character] * 5 * x)
    robot.maze_x_size += x
    # add a new row of size x 5 times per new tile
    for j in range(y):
        for i in range(5):
            robot.map.extend([[character] * 5 * robot.maze_x_size])
    robot.maze_y_size += y


def get_all_values(robot: MazeRobot):
    # increment step
    robot.time_step += 2
    add_to_arr(robot.time_steps_arr, robot.time_step)

    # GPS
    get_gps(robot)
    add_to_arr(robot.robot_position_list, robot.robot_pos.copy())

    # Other Sensors
    get_color_sensor(robot)
    get_cameras_values(robot)
    get_gyro_values(robot)
    # get_lidar(robot)


def map_updater(robot: MazeRobot, x, z):
    x_axis2indx = (abs(robot.maze_x_size) + x) / 12
    z_axis2indx = (abs(robot.maze_y_size) + z) / 12

    print(x_axis2indx - int(x_axis2indx))
    print(z_axis2indx - int(z_axis2indx))

    if (x_axis2indx - int(x_axis2indx) >= 0.47 and x_axis2indx - int(x_axis2indx) <= 0.53 and z_axis2indx - int(
            z_axis2indx) >= 0.47 and z_axis2indx - int(z_axis2indx) <= 0.53):
        stop(robot)
        print("stooop")

        x_axis2indx = int(x_axis2indx)
        z_axis2indx = int(z_axis2indx)

        x_center = (4 * x_axis2indx) + 2
        z_center = (4 * z_axis2indx) + 2

        if robot.map[z_center][x_center] == -1:

            # //////////////Fill Corners////////////////////
            dz = [-1, -1, 1, 1]
            dx = [-1, 1, -1, 1]
            if not robot.start_point:
                corners_value = 5
                robot.start_point = 1
            else:
                pass
                # corners_value = color2num(robot.color_case)
            for i in range(0, 4):
                robot.map[z_center + dz[i]][x_center + dx[i]] = corners_value

            # //////////Fill center and sides//////////////
            robot.map[z_center][x_center] = 0
            dz = [-1, 0, 1, 0]
            dx = [0, -1, 0, 1]
            for i in range(0, 4):
                robot.map[z_center + dz[i]][x_center + dx[i]] = 0

            # //////////print the matrix/////////////////
            for i in robot.map:
                print(i)


# # Avoid holes and swamps by looking at the RBG colour of the camera
def viewColour(robot: MazeRobot, r, g, b):
    color_case = ""

    if (r >= 200) and (g >= 200) and (b >= 200):
        print("White")
        robot.color_case = "white"
    elif (r >= 230) and (g >= 200) and (g <= 240) and (b > 110) and (b <= 160):
        print("Orange")
        robot.color_case = "orange"
    elif (r < 70) and (g < 70) and (b < 70):
        print("Black")
        robot.color_case = "black"


def cam(img):
    rgb = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
    rgb_copy = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)

    # blanc = np.zeros(img.shape[:2] , dtype='uint8')
    # blanc.fill(255)
    # blanc = cv2.cvtColor(blanc, cv2.COLOR_BGR2RGB)

    mask1 = cv2.inRange(rgb, (200, 200, 200), (255, 255, 255))  # masking white
    rgb[mask1 > 0] = (255, 0, 0)
    cv2.imshow("rgb", rgb)

    difference = cv2.subtract(rgb, rgb_copy)
    b, g, r = cv2.split(difference)
    if cv2.countNonZero(b) == 0 and cv2.countNonZero(g) == 0 and cv2.countNonZero(r) == 0:
        print("No Victim detected")

    else:
        print("white")
        # img2 = np.zeros(mask1.shape[:2] , dtype='uint8')
        # img2 = cv2.bitwise_xor(blanc,rgb,mask=mask1)
        # img2 = cv2.bitwise_xor(blanc,rgb)
        # img2 = cv2.cvtColor(img2, cv2.COLOR_BGR2RGB)

        # cv2.imshow("JJ" , img2)

        mask2 = cv2.inRange(rgb_copy, (-50, -50, -50), (80, 80, 80))
        rgb_copy[mask2 > 0] = (0, 0, 255)
        difference = cv2.subtract(rgb, rgb_copy)
        b, g, r = cv2.split(difference)
        if cv2.countNonZero(b) == 0 and cv2.countNonZero(g) == 0 and cv2.countNonZero(r) == 0:
            print("Not found")
        else:
            cv2.imshow("img2", rgb_copy)
            print("victim detected")
    cv2.waitKey(1)
