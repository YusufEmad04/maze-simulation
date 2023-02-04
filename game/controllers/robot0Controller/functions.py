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
        print("current dir: {}".format(robot.current_direction))
        # print("robot pos: ", robot.robot_pos)
        print("next tile: ", robot.next_tile)
        # print_dict(check_walls(robot))
        print("-----------------")
        robot.counter += 1
        if result == -1:
            robot.can_run_simulation = False


def add_to_arr(arr, data):
    arr.append(data)
    if len(arr) >= 3:
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

    if walls["right_move_forward"]:
        index = 128
    elif walls["left_move_forward"]:
        index = 384

    if not index:
        set_left_vel(robot, v)
        set_right_vel(robot, v)
    else:
        rays = []
        for i in robot.lidar_data[2][index - 30: index + 31]:
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
    if robot.color_case == "swamp":
        v = 2

    walls = check_walls(robot)
    index = None

    if walls["right_move_forward"] and walls["left_move_forward"]:
        left_rays_diff = abs(robot.lidar_data[2][384 + 40] - robot.lidar_data[2][384 - 40])
        right_rays_diff = abs(robot.lidar_data[2][128 + 40] - robot.lidar_data[2][128 - 40])

        if left_rays_diff > right_rays_diff:
            index = 128
        else:
            index = 384
    elif walls["right_move_forward"]:
        index = 128
    elif walls["left_move_forward"]:
        index = 384

    if not index:

        set_left_vel(robot, v)
        set_right_vel(robot, v)
    else:
        if index == 128:
            ray1 = robot.lidar_data[2][index - 20]
            ray2 = robot.lidar_data[2][index + 20]

            rays = []
            for i in robot.lidar_data[2][index - 30: index + 31]:
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
            for i in robot.lidar_data[2][index - 30: index + 31]:
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


def move_backward2(robot: MazeRobot, v):
    walls = check_walls(robot)
    index = None

    if walls["right_move_forward"] and walls["left_move_forward"]:
        left_rays_diff = abs(robot.lidar_data[2][128 + 40] - robot.lidar_data[2][128 - 40])
        right_rays_diff = abs(robot.lidar_data[2][384 + 40] - robot.lidar_data[2][384 - 40])

        if left_rays_diff > right_rays_diff:
            index = 384
        else:
            index = 128

    elif walls["right_move_forward"]:
        index = 128
    elif walls["left_move_forward"]:
        index = 384

    if not index:
        set_left_vel(robot, -v)
        set_right_vel(robot, -v)

    else:
        if index == 128:
            ray1 = robot.lidar_data[2][index + 20]
            ray2 = robot.lidar_data[2][index - 20]

            rays = []

            for i in robot.lidar_data[2][index - 30: index + 31]:
                rays.append((i, robot.lidar_data[2].index(i)))

            # get min distance
            min_ray = min(rays, key=lambda x: x[0])

            if min_ray[0] < 11:
                set_left_vel(robot, -v * 0.7)
                set_right_vel(robot, -v)
            else:

                if ray1 > ray2:
                    set_left_vel(robot, -v)
                    set_right_vel(robot, -v * 0.5)
                else:
                    set_left_vel(robot, -v * 0.5)
                    set_right_vel(robot, -v)

        elif index == 384:
            ray1 = robot.lidar_data[2][index - 20]
            ray2 = robot.lidar_data[2][index + 20]

            rays = []

            for i in robot.lidar_data[2][index - 30: index + 31]:
                rays.append((i, robot.lidar_data[2].index(i)))

            # get min distance
            min_ray = min(rays, key=lambda x: x[0])

            if min_ray[0] < 11:
                set_left_vel(robot, -v)
                set_right_vel(robot, -v * 0.7)
            else:

                if ray1 > ray2:
                    set_left_vel(robot, -v * 0.5)
                    set_right_vel(robot, -v)
                else:
                    set_left_vel(robot, -v)
                    set_right_vel(robot, -v * 0.5)


def move_forward3(robot: MazeRobot, v):
    walls = check_walls(robot)
    index = None

    if walls["right_move_forward"]:
        index = 128
    elif walls["left_move_forward"]:
        index = 384

    if not index:
        set_left_vel(robot, v)
        set_right_vel(robot, v)

    else:
        rays = []
        for i in robot.lidar_data[2][index - 30: index + 31]:
            rays.append((i, robot.lidar_data[2].index(i)))

        # get min distance
        min_ray = min(rays, key=lambda x: x[0])
        print(min_ray)

        if index == 128:
            if min_ray[0] > 11.54:
                set_left_vel(robot, v)
                set_right_vel(robot, v * 0.5)
            else:
                set_left_vel(robot, v * 0.5)
                set_right_vel(robot, v)
        else:
            if min_ray[0] > 11.54:
                set_left_vel(robot, v * 0.5)
                set_right_vel(robot, v)
            else:
                set_left_vel(robot, v)
                set_right_vel(robot, v * 0.5)


def turn_right(robot: MazeRobot, v):
    set_left_vel(robot, -v)
    set_right_vel(robot, v)


def turn_left(robot: MazeRobot, v):
    set_left_vel(robot, v)
    set_right_vel(robot, -v)


def turn_90_time_step(robot: MazeRobot, direction="right"):
    if robot.color_case == "swamp":
        x = 217

        speed = 2
    else:
        x = 19 * 2 + 2

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

    print("rotation finished")


def update_dir(robot: MazeRobot):

    if robot.time_step <= 3:
        start = robot.robot_pos.copy()
        t = 1
        while robot.can_run_simulation and t >= 0:
            move_forward(robot, 6)
            t -= 1

            run_simulation(robot, 16)
        set_right_vel(robot, 0)
        set_left_vel(robot, 0)
    else:
        start = robot.robot_position_list[0]

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


def move_one_tile_gps(robot: MazeRobot, half=False):
    if half:
        length = 6
        for_loop_start = 2
        print("----move half----")
    else:
        length = 12
        for_loop_start = 4
        print("----move full----")

    if robot.current_direction in [1, 2]:
        sign = 1
    else:
        sign = -1

    start_x = int(robot.robot_pos[0])
    start_y = int(robot.robot_pos[1])

    print("start_x", start_x)
    print("current_y", start_y)

    if robot.current_direction in (0, 2):
        robot.start_pos[1] = start_y
        robot.start_pos[0] = None

        robot.next_tile = (start_x, start_y + length * sign)

        for i in range(for_loop_start, length + 1):
            if start_y + i % length == 0:
                robot.next_tile = (start_x, start_y + i * sign)
                break

        while abs(robot.start_pos[1] - int(robot.robot_pos[1])) < 3 or int(robot.robot_pos[1]) % length != 0:

            if check_walls(robot)["front_mid_tile"]:
                print("Wall detected")
                return False

            if robot.next_tile in robot.holes:
                while True:
                    set_left_vel(robot, 0)
                    set_right_vel(robot, 0)
                    print("hole \n hole \n hole \n hole")
                    run_simulation(robot)
                return False

            check_camz(robot)

            if robot.can_run_simulation:
                move_forward2(robot, 6.221)
                update_dir(robot)
            run_simulation(robot)

    else:
        robot.start_pos[0] = start_x
        robot.start_pos[1] = None

        robot.next_tile = (start_x + length * sign, start_y)

        for i in range(for_loop_start, length + 1):
            if start_x + i % length == 0:
                robot.next_tile = (start_x + i * sign, start_y)
                break

        while abs(robot.start_pos[0] - int(robot.robot_pos[0])) < 3 or int(robot.robot_pos[0]) % length != 0:

            if check_walls(robot)["front_mid_tile"]:
                print("Wall detected")
                return False

            if robot.next_tile in robot.holes:
                while True:
                    set_left_vel(robot, 0)
                    set_right_vel(robot, 0)
                    print("hole \n hole \n hole \n hole")
                    run_simulation(robot)
                return False

            check_camz(robot)
            if robot.can_run_simulation:
                move_forward2(robot, 6.221)
                update_dir(robot)
            run_simulation(robot)

    print("new gps done at {},  {}".format(*robot.robot_pos))


def stop(robot: MazeRobot, t=10):
    set_left_vel(robot, 0)
    set_right_vel(robot, 0)
    if robot.can_run_simulation:
        run_simulation(robot, step=16 * t)
    else:
        return -1


def get_gps(robot: MazeRobot):
    # robot.robot_pos[0] = robot.gps.getValues()[0] * 100
    # robot.robot_pos[1] = robot.gps.getValues()[2] * 100
    if not robot.initial_map_pos[0] and not robot.initial_map_pos[1]:
        robot.initial_map_pos[0] = robot.gps.getValues()[0] * 100
        robot.initial_map_pos[1] = robot.gps.getValues()[2] * 100
        robot.robot_pos = [0, 0]
    else:
        robot.robot_pos[0] = robot.gps.getValues()[0] * 100 - robot.initial_map_pos[0]
        robot.robot_pos[1] = robot.gps.getValues()[2] * 100 - robot.initial_map_pos[1]


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
    view_colour(robot)


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
def view_colour(robot: MazeRobot):
    color_case = ""
    r, g, b = robot.color_sensor_values

    if (r >= 200) and (g >= 200) and (b >= 200):
        # print("White")
        robot.color_case = "white"
    elif 185 <= r <= 205 and 150 <= g <= 170 and 80 <= b <= 100:
        # print("Orange")
        # TODO Rename swamp to Shorbet 3ads
        robot.color_case = "swamp"
    elif (r < 70) and (g < 70) and (b < 70):
        # print("Black")
        robot.color_case = "black"
        if robot.next_tile:
            robot.holes[robot.next_tile] = True

    return robot.color_case


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
    front_rays_in_range = robot.lidar_data[2][0 - 20] < 17 or robot.lidar_data[2][0 + 20] < 17 or robot.lidar_data[2][
        0] < 17
    back_rays_in_range = robot.lidar_data[2][256 - 20] < 17 and robot.lidar_data[2][256 + 20] < 17 or \
                         robot.lidar_data[2][256] < 17

    right_difference1 = abs(robot.lidar_data[2][128] - robot.lidar_data[2][128 + 40])
    right_difference2 = abs(robot.lidar_data[2][128] - robot.lidar_data[2][128 - 40])

    left_difference1 = abs(robot.lidar_data[2][384] - robot.lidar_data[2][384 + 40])
    left_difference2 = abs(robot.lidar_data[2][384] - robot.lidar_data[2][384 - 40])

    right = right_rays_in_range and right_difference1 < 10 and right_difference2 < 10
    left = left_rays_in_range and left_difference1 < 10 and left_difference2 < 10
    front = front_rays_in_range
    back = back_rays_in_range

    right_navigate = robot.lidar_data[2][128 - 20] < 17 or robot.lidar_data[2][128 + 20] < 17 or robot.lidar_data[2][
        128] < 17
    left_navigate = robot.lidar_data[2][384 - 20] < 17 or robot.lidar_data[2][384 + 20] < 17 or robot.lidar_data[2][
        384] < 17

    front_mid_tile = robot.lidar_data[2][0 - 20] < 10 or robot.lidar_data[2][0] < 9 or robot.lidar_data[2][0 + 20] < 10

    # print("right rays: {}   {}".format(robot.lidar_data[2][128 - 20], robot.lidar_data[2][128 + 20]))

    return {
        "front": front,
        "right_move_forward": right,
        "back": back,
        "left_move_forward": left,
        "right_navigate": right_navigate,
        "left_navigate": left_navigate,
        "front_mid_tile": front_mid_tile
    }


def print_dict(d):
    for k, v in d.items():
        print(k, v)


def send_victim(robot: MazeRobot, vt):
    victim_type = bytes(vt, "utf-8")
    x = int(robot.gps.getValues()[0] * 100)
    y = int(robot.gps.getValues()[2] * 100)
    # TODO Remove prints
    print("vt: {}".format(victim_type))
    message = struct.pack("i i c", x, y, victim_type)
    stop(robot, 100)
    robot.emitter.send(message)
    stop(robot, 64)


def send_end(robot: MazeRobot):
    robot.emitter.send(bytes('E', "utf-8"))


def check_camz(robot: MazeRobot, _detected=False):
    current_x = int(robot.robot_pos[0])
    current_y = int(robot.robot_pos[1])
    not_visited = True

    for i in robot.detected_signs:
        if get_dist(i, (current_x, current_y)) <= math.sqrt(2):
            not_visited = False

    # Detect with right camera
    if moving_cam(robot.right_image) and check_walls(robot)["right_move_forward"] and not_visited:
        _detected = True
        victim_type = full_detection(robot.right_image)
        # print("right")
        send_victim(robot, victim_type)

        x = int(robot.robot_pos[0])
        y = int(robot.robot_pos[1])
        robot.detected_signs[(x, y)] = True

    # Detect with left camera
    if moving_cam(robot.left_image) and check_walls(robot)["left_move_forward"] and not_visited:
        _detected = True
        victim_type = full_detection(robot.left_image)
        # print("left")
        send_victim(robot, victim_type)

        x = int(robot.robot_pos[0])
        y = int(robot.robot_pos[1])
        robot.detected_signs[(x, y)] = True

    return _detected


def check_half(robot: MazeRobot):
    # Determine if move tile or half tile
    front_ray = robot.lidar_data[2][0] < 35
    front_left_ray = robot.lidar_data[2][-20] < 35
    front_right_ray = robot.lidar_data[2][20] < 35

    if front_ray or front_left_ray or front_right_ray:
        return True
    return False


def get_current_tile(robot: MazeRobot):
    current_x = 12 * ((robot.robot_pos[0] + 6) // 12)
    current_y = 12 * ((robot.robot_pos[1] + 6) // 12)
    return [current_x, current_y]


def check_neighbouring_tile(robot: MazeRobot):
    current_tile = get_current_tile(robot)

    tiles = [
        [current_tile[0], current_tile[1] - 12],
        [current_tile[0] + 12, current_tile[1]],
        [current_tile[0], current_tile[1] + 12],
        [current_tile[0] - 12, current_tile[1]],
    ]

    flags = [False, False, False, False]
    for i in range(4):
        for tile in robot.holes:
            if get_dist(tile, tiles[i]) <= 5:
                flags[i] = True

    flags = flags[-robot.current_direction:] + flags[:-robot.current_direction]

    return {
        "front": flags[0],
        "right": flags[1],
        "back": flags[2],
        "left": flags[3]
    }


def navigate(robot: MazeRobot):
    """

    if no wall right.... go right
    if wall right.... go forward
    if wall right, forward.... go left
    if hole...simulate it as wall and change dir when turning
    """

    if not check_walls(robot)["right_navigate"]:
        turn_90_time_step(robot, "right")
        move_one_tile_gps(robot, check_half(robot))
    else:
        if not check_walls(robot)["front"]:
            move_one_tile_gps(robot, check_half(robot))
        else:
            turn_90_time_step(robot, "left")
