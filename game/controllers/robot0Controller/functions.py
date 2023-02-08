import cv2
import math, statistics
import time
import numpy as np
from MazeRobot import MazeRobot
from Camera import moving_cam
from Camera import full_detection
import struct

max_speed = 6.28


def run_simulation(robot: MazeRobot, step=16):
    if robot.can_run_simulation:
        result = robot.robot.step(step)
        get_all_values(robot)
        print("current dir: {}".format(robot.current_direction))
        # print("rounded pos: ", robot.rounded_pos)
        # #print("next tile: ", robot.next_tile)
        # print_dict(check_walls(robot))
        # print_dict(robot.visited_tiles)
        # print_dict(check_neighbouring_tile(robot))
        # print_dict(robot.holes)
        # print_dict(robot.detected_signs)
        # print("robot pos: ", robot.robot_pos)
        print("get current half tile: ", robot.abs_half_tile)
        print("quarter tiles: ", robot.quarter_tiles)
        print("holes :", robot.holes)
        print_dict(check_neighbouring_quarter_tile(robot))
        print("right_navigate:", check_walls(robot)["right_navigate"])
        print("left_navigate:", check_walls(robot)["left_navigate"])
        # print_dict(get_neighbouring_quarter_tile(robot))
        # print_dict(robot.visited_quarters)
        # print("quarter neighbours: ", check_neighbouring_quarter_tile(robot))
        # print_dict(robot.visited_quarters)
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
        # TODO Make sure to submit correct x
        # x = 217
        x = 70

        speed = 2
    else:
        x = 19 * 2 + 2

        speed = 3.48985044

    if direction == "right":
        robot.current_direction = (robot.current_direction + 1) % 4
    elif direction == "left":
        speed = -speed
        robot.current_direction = (robot.current_direction - 1) % 4

    # detected = False

    for i in range(x):

        check_camz(robot)
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
        # print("----move half----")
    else:
        length = 12
        for_loop_start = 4
        # print("----move full----")

    if robot.current_direction in [1, 2]:
        sign = 1
    else:
        sign = -1

    start_x = int(robot.robot_pos[0])
    start_y = int(robot.robot_pos[1])

    # print("start_x", start_x)
    # print("current_y", start_y)

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
                return False

            check_camz(robot)

            if robot.can_run_simulation:

                if robot.should_move_back:
                    move_back_hole(robot)
                    robot.should_move_back = False
                    return False

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
                return False

            check_camz(robot)
            if robot.can_run_simulation:

                if robot.should_move_back:
                    move_back_hole(robot)
                    robot.should_move_back = False
                    return False

                move_forward2(robot, 6.221)
                update_dir(robot)
            run_simulation(robot)

    # print("new gps done at {},  {}".format(*robot.robot_pos))
    robot.visited_tiles[robot.rounded_pos] += 1


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

    robot.rounded_pos = (
        round_to_12(robot.robot_pos[0]),
        round_to_12(robot.robot_pos[1])
    )

    if robot.rounded_pos not in robot.visited_tiles:
        robot.visited_tiles[robot.rounded_pos] = 0

    get_current_tile2(robot)
    quarter_tiles = get_quarter_tiles_around(robot, robot.abs_half_tile)

    # for q_t in quarter_tiles:
    #     if q_t not in robot.visited_quarters:
    #         robot.visited_quarters[q_t[0]] = True

    for key in quarter_tiles:
        tile = quarter_tiles[key]
        if tile not in robot.visited_quarters:
            robot.visited_quarters[tile] = True


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


def move_back_hole(robot: MazeRobot):

    start_x = int(robot.robot_pos[0])
    start_y = int(robot.robot_pos[1])

    print("BACKING")

    if robot.current_direction in (0, 2):
        robot.start_pos[1] = start_y
        robot.start_pos[0] = None

        while int(robot.robot_pos[1]) % 12 != 0:

            if robot.can_run_simulation:
                move_backward2(robot, 6.221)
            run_simulation(robot)
        # stop(robot, 100)

    else:
        robot.start_pos[0] = start_x
        robot.start_pos[1] = None

        while int(robot.robot_pos[0]) % 12 != 0:

            if robot.can_run_simulation:
                move_backward2(robot, 6.221)
            run_simulation(robot)
        # stop(robot, 100)


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
        add_hole(robot)
        # if robot.next_tile:
        #     add_hole = True
        #     for hole in robot.holes:
        #         if get_dist(hole, robot.next_tile) < 6:
        #             add_hole = False
        #             break
        #
        #     if add_hole:
        #         robot.holes[robot.next_tile] = True
        #     move_back_hole(robot)

    return robot.color_case


def add_hole(robot: MazeRobot):
    if robot.current_direction in (1, 2):
        sign = 1
    else:
        sign = -1

    if robot.current_direction in (0, 2):
        hole_tile = (robot.abs_half_tile[0], robot.abs_half_tile[1] + sign * 12)
    else:
        hole_tile = (robot.abs_half_tile[0] + sign * 12, robot.abs_half_tile[1])

    hole_quarters = get_quarter_tiles_around(robot, hole_tile)

    for key in hole_quarters:
        tile = hole_quarters[key]
        if tile not in robot.holes:
            robot.holes[tile] = True

    # move_back_hole(robot)
    robot.should_move_back = True

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
     # front_rays_in_range = robot.lidar_data[2][0 - 20] < 17 or robot.lidar_data[2][0 + 20] < 17 or robot.lidar_data[2][
    #     0] < 17
    # back_rays_in_range = robot.lidar_data[2][256 - 20] < 17 or robot.lidar_data[2][256 + 20] < 17 or \
    #                      robot.lidar_data[2][256] < 17

    front_rays_in_range = False
    back_rays_in_range = False

    for i in range(-20, 20):
        if robot.lidar_data[2][0 + i] < 17:
            front_rays_in_range = True
            break

    for i in range(-20, 20):
        if robot.lidar_data[2][256 + i] < 17:
            back_rays_in_range = True
            break

    right_difference1 = abs(robot.lidar_data[2][128] - robot.lidar_data[2][128 + 40])
    right_difference2 = abs(robot.lidar_data[2][128] - robot.lidar_data[2][128 - 40])

    left_difference1 = abs(robot.lidar_data[2][384] - robot.lidar_data[2][384 + 40])
    left_difference2 = abs(robot.lidar_data[2][384] - robot.lidar_data[2][384 - 40])

    right = right_rays_in_range and right_difference1 < 10 and right_difference2 < 10
    left = left_rays_in_range and left_difference1 < 10 and left_difference2 < 10
    front = front_rays_in_range
    back = back_rays_in_range

    # right_navigate = robot.lidar_data[2][128 - 20] < 17 or robot.lidar_data[2][128 + 20] < 17 or robot.lidar_data[2][
    #     128] < 17
    # left_navigate = robot.lidar_data[2][384 - 20] < 17 or robot.lidar_data[2][384 + 20] < 17 or robot.lidar_data[2][
    #     384] < 17
    #
    front_mid_tile = robot.lidar_data[2][0 - 20] < 10 or robot.lidar_data[2][0] < 9 or robot.lidar_data[2][0 + 20] < 10


    right_navigate = False
    for i in range(-20, 21):
        if robot.lidar_data[2][128 + i] < 17:
            right_navigate = True
            break

    left_navigate = False
    for i in range(-20, 21):
        if robot.lidar_data[2][384 + i] < 17:
            left_navigate = True
            break
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

    # for i in robot.detected_signs:
    #     if get_dist(i, (current_x, current_y)) <= math.sqrt(2):
    #         not_visited = False

    not_visited = not value_in_dict(robot.detected_signs, (current_x, current_y))

    # Detect with right camera
    if moving_cam(robot.right_image) and check_walls(robot)["right_navigate"] and not_visited:
        _detected = True
        victim_type = full_detection(robot.right_image)
        # print("right")
        send_victim(robot, victim_type)

        x = int(robot.robot_pos[0])
        y = int(robot.robot_pos[1])
        robot.detected_signs[(x, y)] = True

    # Detect with left camera
    if moving_cam(robot.left_image) and check_walls(robot)["left_navigate"] and not_visited:
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


def get_current_tile2(robot: MazeRobot):
    if close_to_12(robot.robot_pos[0]):
        current_x = round_to_12(robot.robot_pos[0])
    else:
        current_x = round_to_6(robot.robot_pos[0])

    if close_to_12(robot.robot_pos[1]):
        current_y = round_to_12(robot.robot_pos[1])
    else:
        current_y = round_to_6(robot.robot_pos[1])

    robot.abs_half_tile = (current_x, current_y)

    return [current_x, current_y]


def get_quarter_tiles_around(robot: MazeRobot, robot_pos):

    quarter_tiles_dict = {
        "top_left": (robot_pos[0] - 3, robot_pos[1] - 3),
        "bottom_left": (robot_pos[0] - 3, robot_pos[1] + 3),
        "top_right": (robot_pos[0] + 3, robot_pos[1] - 3),
        "bottom_right": (robot_pos[0] + 3, robot_pos[1] + 3)
    }

    robot.quarter_tiles = quarter_tiles_dict

    return quarter_tiles_dict


def check_neighbour_holes(robot: MazeRobot):
    neighbours = get_neighbouring_quarter_tile(robot)

    neighbours_dict = {
        "front": False,
        "right": False,
        "back": False,
        "left": False
    }

    for side in neighbours:
        for q_t in neighbours[side]:
            if q_t in robot.holes:
                neighbours_dict[side] = True

    return neighbours_dict


def check_neighbouring_tile(robot: MazeRobot):
    current_tile = get_current_tile(robot)

    tiles = [
        ((current_tile[0], current_tile[1] - 12), "front"),
        ((current_tile[0] + 12, current_tile[1]), "right"),
        ((current_tile[0], current_tile[1] + 12), "back"),
        ((current_tile[0] - 12, current_tile[1]), "left"),
    ]

    neighbours_dict = {
        "front": True,
        "right": True,
        "back": True,
        "left": True
    }

    # for tile in tiles:
    #     for hole in robot.holes:
    #         if get_dist(tile[0], hole) <= 3:
    #             neighbours_dict[tile[1]] = False

    for tile in tiles:
        neighbours_dict[tile[1]] = not value_in_dict(robot.holes, tile[0]) and not value_in_dict(robot.visited_tiles, tile[0])

    if robot.current_direction == 0:
        return neighbours_dict
    if robot.current_direction == 1:
        return {
            "front": neighbours_dict["right"],
            "right": neighbours_dict["back"],
            "back": neighbours_dict["left"],
            "left": neighbours_dict["front"]
        }
    elif robot.current_direction == 2:
        return {
            "front": neighbours_dict["back"],
            "right": neighbours_dict["left"],
            "back": neighbours_dict["front"],
            "left": neighbours_dict["right"]
        }
    elif robot.current_direction == 3:
        return {
            "front": neighbours_dict["left"],
            "right": neighbours_dict["front"],
            "back": neighbours_dict["right"],
            "left": neighbours_dict["back"]
        }


def get_neighbouring_quarter_tile(robot: MazeRobot):
    top_left = robot.quarter_tiles["top_left"]
    bottom_left = robot.quarter_tiles["bottom_left"]
    top_right = robot.quarter_tiles["top_right"]
    bottom_right = robot.quarter_tiles["bottom_right"]

    neighbours = {
        "front": (
            (top_left[0], top_left[1] - 6),
            (top_right[0], top_right[1] - 6)
        ),
        "back": (
            (bottom_left[0], bottom_left[1] + 6),
            (bottom_right[0], bottom_right[1] + 6)
        ),
        "left": (
            (top_left[0] - 6, top_left[1]),
            (bottom_left[0] - 6, bottom_left[1])
        ),
        "right": (
            (top_right[0] + 6, top_right[1]),
            (bottom_right[0] + 6, bottom_right[1])
        )
        }

    if robot.current_direction == 0:
        return neighbours
    if robot.current_direction == 1:
        return {
            "front": neighbours["right"],
            "back": neighbours["left"],
            "left": neighbours["front"],
            "right": neighbours["back"]
        }
    elif robot.current_direction == 2:
        return {
            "front": neighbours["back"],
            "back": neighbours["front"],
            "left": neighbours["right"],
            "right": neighbours["left"]
        }
    elif robot.current_direction == 3:
        return {
            "front": neighbours["left"],
            "back": neighbours["right"],
            "left": neighbours["back"],
            "right": neighbours["front"]
        }


def check_neighbouring_quarter_tile(robot: MazeRobot):
    neighbours = get_neighbouring_quarter_tile(robot)

    neighbours_dict = {
        "front": True,
        "right": True,
        "back": True,
        "left": True
    }

    for side in neighbours:
        if neighbours[side][0] in robot.visited_quarters and neighbours[side][1] in robot.visited_quarters:
            neighbours_dict[side] = False
        for q_t in neighbours[side]:
            if q_t in robot.holes:
                neighbours_dict[side] = False

    return neighbours_dict

def right_left_back_walls(robot: MazeRobot):

    right = check_walls(robot)["right_navigate"] or not check_neighbouring_quarter_tile(robot)["right"] or check_neighbour_holes(robot)["right"]
    left = check_walls(robot)["left_navigate"] or not check_neighbouring_quarter_tile(robot)["left"] or check_neighbour_holes(robot)["left"]
    back = check_walls(robot)["back"] or not check_neighbouring_quarter_tile(robot)["back"] or check_neighbour_holes(robot)["back"]

    print("right_____", right)
    print("left______", left)
    print("back______", back)
    print("----------catcat-------")

    return right and left and back


def navigate(robot: MazeRobot):
    """

    if no wall right.... go right
    if wall right.... go forward
    if wall right, forward.... go left
    if hole...simulate it as wall and change dir when turning
    """

    if not (check_walls(robot)["right_navigate"]) and check_neighbouring_tile(robot)["right"]:
        print("cat right")
        turn_90_time_step(robot, "right")
        # stop(robot,10)

        move_one_tile_gps(robot, check_half(robot))
        # stop(robot,10)

    else:
        if check_walls(robot)["front"] or check_neighbour_holes(robot)["front"]:
            if robot.visited_tiles[robot.rounded_pos] % 2 != 0:
                turn_90_time_step(robot, "left")
            else:
                turn_90_time_step(robot, "right")

        elif (not check_walls(robot)["front"] and check_neighbouring_tile(robot)["front"]) or right_left_back_walls(robot):
            print("cat front")

            move_one_tile_gps(robot, check_half(robot))
            # stop(robot, 10)

        else:
            print("elseeeeeeeeeeeeeee")
            turn_90_time_step(robot, "left")
def navigate2(robot:MazeRobot):
    """
    if no wall right.... go right
    if wall right.... go forward
    if wall right, forward.... go left
    if hole...simulate it as wall and change dir when turning
    """

    if not(check_walls(robot)["right_navigate"]) and check_neighbouring_quarter_tile(robot)["right"]:
        print("Turn Right")
        turn_90_time_step(robot, "right")

        move_one_tile_gps(robot, True)
    else:
        if check_walls(robot)["front"] or check_neighbour_holes(robot)["front"]:
            turn_90_time_step(robot, "left")

        elif (not check_walls(robot)["front"] and check_neighbouring_quarter_tile(robot)["front"]) or right_left_back_walls(robot):
           print("Move Front")

           move_one_tile_gps(robot, True)

        else:
            print("Turn Left")
            turn_90_time_step(robot, "left")

def round_to_12(x):
    return 12 * ((x + 6) // 12)


def round_to_6(x):
    return 6 * ((x + 3) // 6)


def close_to_12(x):
    if x % 12 <= 2 or x % 12 >= 10:
        return True
    return False


def value_in_dict(d, value):
    found = False
    for i in d:
        if get_dist(i, value) <= 3:
            found = True

    return found