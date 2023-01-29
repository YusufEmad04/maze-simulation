import cv2
import math, statistics
import time
import numpy as np
from MazeRobot import MazeRobot
from Camera import moving_cam
from Camera import full_detection
import struct
from turtle import position  # TODO DELETE IT

max_speed = 6.28


def run_simulation(robot: MazeRobot, step=16):
    if robot.can_run_simulation:
        result = robot.robot.step(step)
        get_all_values(robot)
        robot.counter += 1
        if result == -1:
            robot.can_run_simulation = False


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
    robot.left_wheel.setVelocity(v)


def set_right_vel(robot: MazeRobot, v):
    robot.right_wheel.setVelocity(v)


def move_forward(robot: MazeRobot, v):

    walls = check_walls(robot)
    index = None

    if walls["right"]:
        index = 128
    elif walls["left"]:
        index = 384

    if not index:
        set_left_vel(robot, v)
        set_right_vel(robot, v)
    else:
        rays = []
        for i in robot.lidar_data[2][index - 30 : index + 31]:
            rays.append((i, robot.lidar_data[2].index(i)))

        # get min distance
        min_ray = min(rays, key=lambda x: x[0])
        ratio = abs(index - min_ray[1]) / 30

        if index == 128:

            if min_ray[1] > index:
                set_left_vel(robot, v)
                set_right_vel(robot, v * (1 - ratio))
            else:
                set_left_vel(robot, v * (1 - ratio))
                set_right_vel(robot, v)

        elif index == 384:
            if min_ray[1] > index:
                set_left_vel(robot, v)
                set_right_vel(robot, v * (1 - ratio))
            else:
                set_left_vel(robot, v * (1 - ratio))
                set_right_vel(robot, v)


def move_forward2(robot: MazeRobot, v):
    walls = check_walls(robot)
    index = None

    if walls["right"] and walls["left"]:
        left_rays_diff = abs(robot.lidar_data[2][384 + 20] - robot.lidar_data[2][384 - 20])
        right_rays_diff = abs(robot.lidar_data[2][128 + 20] - robot.lidar_data[2][128 - 20])

        if left_rays_diff > right_rays_diff:
            index = 128
        else:
            index = 384
    elif walls["right"]:
        index = 128
    elif walls["left"]:
        index = 384

    if not index:

        set_left_vel(robot, v)
        set_right_vel(robot, v)
    else:
        if index == 128:
            ray1 = robot.lidar_data[2][index - 20]
            ray2 = robot.lidar_data[2][index + 20]

            rays = []
            for i in robot.lidar_data[2][index - 30 : index + 31]:
                rays.append((i, robot.lidar_data[2].index(i)))

            # get min distance
            min_ray = min(rays, key=lambda x: x[0])

            if min_ray[0] < 11:
                set_left_vel(robot, v * 0.7)
                set_right_vel(robot, v)

            else:

                if ray1 > ray2:
                    set_left_vel(robot, v)
                    set_right_vel(robot, v * 0.5)
                    # set_right_vel(robot, v * (1 - ratio))
                else:
                    # set_left_vel(robot, v * (1 - ratio))
                    set_left_vel(robot, v * 0.5)
                    set_right_vel(robot, v)


        elif index == 384:
            ray1 = robot.lidar_data[2][index - 20]
            ray2 = robot.lidar_data[2][index + 20]

            rays = []
            for i in robot.lidar_data[2][index - 30 : index + 31]:
                rays.append((i, robot.lidar_data[2].index(i)))

            # get min distance
            min_ray = min(rays, key=lambda x: x[0])

            if min_ray[0] < 11:
                set_left_vel(robot, v)
                set_right_vel(robot, v * 0.7)

            else:

                if ray1 > ray2:
                    set_left_vel(robot, v)
                    # set_right_vel(robot, v * (1 - ratio))
                    set_right_vel(robot, v * 0.5)
                else:
                    # set_left_vel(robot, v * (1 - ratio))
                    set_left_vel(robot, v * 0.5)
                    set_right_vel(robot, v)

def move_forward3(robot: MazeRobot, v):

    walls = check_walls(robot)
    index = None

    if walls["right"]:
        index = 128
    elif walls["left"]:
        index = 384

    if not index:
        set_left_vel(robot, v)
        set_right_vel(robot, v)

    else:
        rays = []
        for i in robot.lidar_data[2][index - 30 : index + 31]:
            rays.append((i, robot.lidar_data[2].index(i)))

        # get min distance
        min_ray = min(rays, key=lambda x: x[0])
        print(min_ray)

        if index == 128:
            if min_ray[0] > 11.54:
                set_left_vel(robot, v)
                set_right_vel(robot, v*0.5)
            else:
                set_left_vel(robot, v*0.5)
                set_right_vel(robot, v)
        else:
            if min_ray[0] > 11.54:
                set_left_vel(robot, v*0.5)
                set_right_vel(robot, v)
            else:
                set_left_vel(robot, v)
                set_right_vel(robot, v*0.5)

def turn_right(robot: MazeRobot, v):
    set_left_vel(robot, -v)
    set_right_vel(robot, v)


def turn_left(robot: MazeRobot, v):
    set_left_vel(robot, v)
    set_right_vel(robot, -v)


def turn_90_time_step_with_camera(robot: MazeRobot, direction):
    x = 18
    z = 10
    if robot.color_case == "orange":
        speed = 5.4464
    else:
        speed = 3.4898

    if direction == "right":
        speed = -speed
        robot.current_direction = (robot.current_direction + 1) % 4
    elif direction == "left":
        robot.current_direction = (robot.current_direction - 1) % 4

    detected = False
    y = 0
    while robot.robot.step(32) != -1 and x >= 0:
        if y > 0:
            y -= 1
            stop(robot)
        else:
            get_all_values(robot)
            print("gyro values: ", robot.gyro_values)
            x -= 1
            turn_left(robot, speed)

            if not detected:
                if moving_cam(robot.left_image):
                    victim_type = full_detection(robot.left_image)
                    detected = True
                    print(f"Victim type = {victim_type}")  # send to the receiver
                    y = 10
    while robot.robot.step(32) != -1 and z >= 0:
        z -= 1
        stop(robot)


def turn_90_time_step(robot: MazeRobot, direction="right"):
    x = 19 * 2 + 2
    # if robot.color_case == "orange":
    #     speed = 5.4464
    # else:
    #     speed = 3.4868

    speed = 3.48985044

    if direction == "right":
        robot.current_direction = (robot.current_direction + 1) % 4
    elif direction == "left":
        speed = -speed
        robot.current_direction = (robot.current_direction - 1) % 4

    detected = False

    for i in range(x):

        if not detected:
            if check_camz(robot):
                detected = True
                # stop(robot, 150)

        turn_left(robot, speed)

        if robot.can_run_simulation:
            run_simulation(robot, step=16)

    if direction == "right":
        update_hole(robot, 1)
    else:
        update_hole(robot, 2)

    print("rotation finished")
    # stop(robot, 120)

    # TEMP Add a break after turn
    # while robot.robot.step(32) != -1 and x >= -5:
    #     stop(robot)
    #     x -= 1


def initialize_dir(robot: MazeRobot):
    start = robot.robot_pos.copy()
    t = 1
    while robot.can_run_simulation and t >= 0:
        move_forward(robot, 6)
        t -= 1

        run_simulation(robot, 16)

    x_diff = robot.robot_pos[0] - start[0]
    y_diff = robot.robot_pos[1] - start[1]

    if abs(x_diff) > abs(y_diff):
        if x_diff > 0:
            robot.current_direction = 1
        else:
            robot.current_direction = 3
    else:
        if y_diff > 0:
            robot.current_direction = 2
        else:
            robot.current_direction = 0


def move_one_tile(robot: MazeRobot):
    x = 28 * 2 + 2
    if robot.color_case == "orange":
        speed = 5.4464
    else:
        speed = 3.4898

    detected = False

    for i in range(x):

        if not detected:
            if check_camz(robot):
                detected = True

        move_forward2(robot, 6.221)

        if robot.can_run_simulation:
            run_simulation(robot, step=16)
        else:
            return -1

    print("tile finished")
    update_hole(robot)
    # stop(robot, 150)

    return True


def move_one_tile_gps(robot: MazeRobot):
    """
    0: front
    1: right
    2: back
    3: left
    """
    # Get wanted position
    if robot.current_direction in [1, 2]:
        sign = 1
    else:
        sign = -1

    detected = False

    if robot.current_direction in [0, 2]:
        wanted_y = robot.abs_pos[1] + 12 * sign

        while abs(robot.robot_pos[1] - wanted_y) >= 0.4:
            # Detect victim once
            if not detected:
                if check_camz(robot):
                    detected = True

            # Move one tile
            if robot.can_run_simulation:
                move_forward2(robot, 6.221)
                update_hole(robot)
            run_simulation(robot)

        robot.abs_pos = (robot.abs_pos[0], wanted_y)
        print("Arrived Y")
        # stop(robot, 150)

    else:
        wanted_x = robot.abs_pos[0] + 12 * sign

        while abs(robot.robot_pos[0] - wanted_x) >= 0.4:
            # Detect victim once
            if not detected:
                if check_camz(robot):
                    detected = True

            # Move one tile
            if robot.can_run_simulation:
                move_forward2(robot, 6.221)
            run_simulation(robot)

        robot.abs_pos = (wanted_x, robot.abs_pos[1])
        print("Arrived x")
        update_hole(robot)
        # stop(robot, 150)


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
            if victim_type != "N" and detected == False:
                detected = True
                print(f"Victim type = {victim_type}")  # send to the receiver
                stop(robot)
                # time.sleep(0.7)
                '''
                loop for 23 times
                
                
                
                '''
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


def stop(robot: MazeRobot, t=10):
    set_left_vel(robot, 0)
    set_right_vel(robot, 0)
    if robot.can_run_simulation:
        run_simulation(robot, step=16 * t)
    else:
        return -1


def get_gps(robot: MazeRobot):
    robot.robot_pos[0] = robot.gps.getValues()[0] * 100
    robot.robot_pos[1] = robot.gps.getValues()[2] * 100
    if robot.abs_pos == [-1, -1]:
        robot.abs_pos = robot.robot_pos.copy()


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
    robot.left_image = cv2.cvtColor(robot.left_image, cv2.COLOR_BGRA2BGR)
    robot.right_image = np.frombuffer(robot.right_camera.getImage(), np.uint8).reshape(
        (robot.right_camera.getHeight(), robot.right_camera.getWidth(), 4))
    robot.right_image = cv2.cvtColor(robot.right_image, cv2.COLOR_BGRA2BGR)

    color_sensor_image = robot.color_sensor.getImage()
    robot.color_sensor_values[0] = robot.color_sensor.imageGetRed(color_sensor_image, 1, 0, 0)
    robot.color_sensor_values[1] = robot.color_sensor.imageGetGreen(color_sensor_image, 1, 0, 0)
    robot.color_sensor_values[2] = robot.color_sensor.imageGetBlue(color_sensor_image, 1, 0, 0)


def get_lidar(robot: MazeRobot):
    # Loop on lidar data and add it to a 2D array
    # empty the array
    robot.lidar_data = []
    range_image = robot.lidar.getRangeImage()
    for layer in range(4):
        robot.lidar_data.append([])
        for point in range(512):
            robot.lidar_data[layer].append(range_image[layer * 512 + point] * 100)
    # lidar_group_values(robot)
    for point in range(512):
        robot.lidar_data[2][point] = robot.lidar_data[2][point] * 2.54 * math.cos(0.0333333333333)


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
    get_lidar(robot)
    viewColour(robot)


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
def viewColour(robot: MazeRobot):
    color_case = ""
    r, g, b = robot.color_sensor_values

    if (r >= 200) and (g >= 200) and (b >= 200):
        print("White")
        robot.color_case = "white"
    elif (r >= 230) and (g >= 200) and (g <= 240) and (b > 110) and (b <= 160):
        print("Orange")
        robot.color_case = "orange"
    elif (r < 70) and (g < 70) and (b < 70):
        print("Black")
        robot.color_case = "black"
        set_hole_location(robot)

    return robot.color_case


def set_hole_location(robot: MazeRobot):
    print("CHANGED\n____________")
    robot.hole_direction_pos[0] = 0

    if robot.current_direction == 0:
        hole_pos = [robot.robot_pos[0], robot.robot_pos[1] - 12]
    elif robot.current_direction == 1:
        hole_pos = [robot.robot_pos[0] + 12, robot.robot_pos[1]]
    elif robot.current_direction == 2:
        hole_pos = [robot.robot_pos[0], robot.robot_pos[1] + 12]
    else:
        hole_pos = [robot.robot_pos[0] - 12, robot.robot_pos[1]]

    # if get_dist(hole_pos, robot.hole_direction_pos[1]) >= 0.5:
    robot.hole_direction_pos[1] = hole_pos
    avoid_hole(robot)



# TODO Sequence to break move_one_tile when hole found
def avoid_hole(robot: MazeRobot):
    pass


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


def check_walls(robot: MazeRobot):
    right_rays_in_range = 3 < robot.lidar_data[2][128 - 20] < 17 and 3 < robot.lidar_data[2][128 + 20] < 17
    left_rays_in_range = 3 < robot.lidar_data[2][384 - 20] < 17 and 3 < robot.lidar_data[2][384 + 20] < 17
    front_rays_in_range = robot.lidar_data[2][0 - 20] < 17 and robot.lidar_data[2][0 + 20] < 17
    back_rays_in_range = 3 < robot.lidar_data[2][256 - 20] < 17 and 3 < robot.lidar_data[2][256 + 20] < 17

    right_difference1 = abs(robot.lidar_data[2][128] - robot.lidar_data[2][128 + 40])
    right_difference2 = abs(robot.lidar_data[2][128] - robot.lidar_data[2][128 - 40])

    left_difference1 = abs(robot.lidar_data[2][384] - robot.lidar_data[2][384 + 40])
    left_difference2 = abs(robot.lidar_data[2][384] - robot.lidar_data[2][384 - 40])

    front_difference1 = abs(robot.lidar_data[2][0] - robot.lidar_data[2][0 + 40])
    front_difference2 = abs(robot.lidar_data[2][0] - robot.lidar_data[2][0 - 40])

    back_difference1 = abs(robot.lidar_data[2][256] - robot.lidar_data[2][256 + 40])
    back_difference2 = abs(robot.lidar_data[2][256] - robot.lidar_data[2][256 - 40])

    right = right_rays_in_range and right_difference1 < 10 and right_difference2 < 10
    left = left_rays_in_range and left_difference1 < 10 and left_difference2 < 10
    front = front_rays_in_range
    back = back_rays_in_range

    # print("right rays: {}   {}".format(robot.lidar_data[2][128 - 20], robot.lidar_data[2][128 + 20]))

    return {
        "front": front,
        "right": right,
        "back": back,
        "left": left
    }


def check_holes(robot: MazeRobot):
    front = robot.hole_direction_pos[0] == 0
    right = robot.hole_direction_pos[0] == 1
    back = robot.hole_direction_pos[0] == 2
    left = robot.hole_direction_pos[0] == 3

    return {
        "front": front,
        "right": right,
        "back": back,
        "left": left
    }


def print_dict(d):
    for k, v in d.items():
        print(k, v)


def print_lidar_triples(robot: MazeRobot):
    front_ray = (robot.lidar_data[2][0], 0)
    right_ray = (robot.lidar_data[2][128], 128)
    back_ray = (robot.lidar_data[2][256], 256)
    left_ray = (robot.lidar_data[2][384], 384)

    print("front {}   {}   {}, range: {}".format(*get_ray_triple(robot, front_ray)))
    print("right {}   {}   {}, range: {}".format(*get_ray_triple(robot, right_ray)))
    print("back  {}   {}   {}, range: {}".format(*get_ray_triple(robot, back_ray)))
    print("left  {}   {}   {}, range: {}".format(*get_ray_triple(robot, left_ray)))
    print("-------------------------")


def get_ray_triple(robot: MazeRobot, ray):
    if 9 <= ray[0] <= 15:
        range = 50
    else:
        range = 20

    return robot.lidar_data[2][ray[1] - range], robot.lidar_data[2][ray[1]], robot.lidar_data[2][ray[1] + range], range


def send_victim(robot: MazeRobot, vt):
    victim_type = bytes(vt, "utf-8")
    x = int(robot.gps.getValues()[0] * 100)
    y = int(robot.gps.getValues()[2] * 100)
    # TODO Remove prints
    # print("X: {}, Y: {}".format(x, y))
    print("vt: {}".format(victim_type))
    message = struct.pack("i i c", x, y, victim_type)
    stop(robot, 100)
    robot.emitter.send(message)
    stop(robot, 64)


def turn_90_with_lidar(robot: MazeRobot, direction):
    if direction == "right":
        speed = -5
    else:
        speed = 5

    for i in range(17):
        set_left_vel(robot, 6)
        set_right_vel(robot, -6)
        run_simulation(robot, 16)


    rays = []
    for i in robot.lidar_data[2]:
        rays.append(i)


    pass


def are_equal(a, b):
    for i in range(len(a)):
        if abs(a[i] - b[i]) > 0.5:
            return False
    return True


def send_end(robot: MazeRobot):
    robot.emitter.send(bytes('E', "utf-8"))


def check_camz(robot: MazeRobot, _detected=False):



    # Detect with right camera
    if moving_cam(robot.right_image) and check_walls(robot)["right"]:
        _detected = True
        victim_type = full_detection(robot.right_image)
        # print("right")
        send_victim(robot, victim_type)

    # Detect with left camera
    if moving_cam(robot.left_image) and check_walls(robot)["left"]:
        _detected = True
        victim_type = full_detection(robot.left_image)
        # print("left")
        send_victim(robot, victim_type)

    return _detected

def update_hole(robot: MazeRobot, dir=0):
    # dir = ["front", "right", " left"]
    if get_dist(robot.robot_pos, robot.hole_direction_pos[1]) >= 18:
        robot.hole_direction_pos[0] = -1
    elif robot.hole_direction_pos[0] != -1:
        if dir == 0:
            pass
        if dir == 1:
            robot.hole_direction_pos[0] = (robot.hole_direction_pos[0] - 1) % 4
        else:
            robot.hole_direction_pos[0] = (robot.hole_direction_pos[0] + 1) % 4
def navigate(robot: MazeRobot):
    """

    if no wall right.... go right
    if wall right.... go forward
    if wall right, forward.... go left
    if hole...simulate it as wall and change dir when turning
    """
    print("DATA:", robot.hole_direction_pos)
    print(check_holes(robot))
    if not check_walls(robot)["right"] and not check_holes(robot)["right"]:
        turn_90_time_step(robot, "right")
        # move_one_tile_gps(robot)
        move_one_tile(robot)
    else:
        if not check_walls(robot)["front"] and not check_holes(robot)["front"]:
            move_one_tile_gps(robot)
        else:
            turn_90_time_step(robot, "left")
