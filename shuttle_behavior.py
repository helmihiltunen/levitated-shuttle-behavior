"""
Tampere University
Faculty of Engineering and Natural Sciences
Bachelor's thesis project
Author: Helmi Hiltunen
"""

from pmclib import xbot_commands as bot
from pmclib import system_commands as sys
from pmclib import pmc_types as pm
import time
import math
import random

TOTAL_SHUTTLES = 8
PUSHER_RATIO = 0.25
LOOP_DELAY = 0.1
BASE_SPEED = 0.2
SAFE_DISTANCE = 0.20
CAUTION_DISTANCE = 0.10
ASSIGNMENT_MODE = "even"   # "even" or "random"
MAX_ACCEL = 1.0
GOAL_TOLERANCE = 0.01
CMD_LABEL = 1

SEGMENT_SIZE = 0.24
OFFSET = 0.12

# X-coordinates for each column
X_COL_1 = OFFSET
X_COL_2 = OFFSET + SEGMENT_SIZE
X_COL_3 = OFFSET + 2 * SEGMENT_SIZE
X_COL_4 = OFFSET + 3 * SEGMENT_SIZE

#Start and goal rows
SPECIAL_ROWS = [0, 1, 6, 7, 12, 13, 18, 19]

class ConnectToSim:
    def __init__(self):
        #initializing and connecting to simulation in pmst
        sys.auto_search_and_connect_to_pmc()
        sys.gain_mastership()

class BehaviorLogic:
    def assign_behaviors(self, shuttles, pusher_ratio, mode="even"):
        behaviors = {}
        n = len(shuttles)
        pusher_count = int(n * pusher_ratio)

        if pusher_count <= 0:
            for shuttle_id in shuttles:
                behaviors[shuttle_id] = "passive"
            return behaviors

        if mode == "random":
            pusher_ids = set(random.sample(shuttles, pusher_count))

        elif mode == "even":
            if pusher_count == 1:
                indices = [0]
            else:
                step = (n - 1) / (pusher_count - 1)
                indices = [round(i * step) for i in range(pusher_count)]

            pusher_ids = set(shuttles[i] for i in indices)

        else:
            raise ValueError("mode must be 'even' or 'random'")

        for shuttle_id in shuttles:
            if shuttle_id in pusher_ids:
                behaviors[shuttle_id] = "pusher"
            else:
                behaviors[shuttle_id] = "passive"

        return behaviors



    def find_velocity(self, behavior_type, neighbor_distance):
    #Changing agent speed to approprite one for the moment
        if behavior_type == "passive":
            if neighbor_distance < CAUTION_DISTANCE:
                new_speed = BASE_SPEED * 0.25

            elif neighbor_distance < SAFE_DISTANCE:
                new_speed = BASE_SPEED * 0.5

            else:
                new_speed = BASE_SPEED

        elif behavior_type == "pusher":
            if neighbor_distance < CAUTION_DISTANCE:
                new_speed = BASE_SPEED * 0.75

            elif neighbor_distance < SAFE_DISTANCE:
                new_speed = BASE_SPEED * 1.2

            else:
                new_speed = BASE_SPEED

        return new_speed

def get_xy(shuttle_id):
    status = bot.get_xbot_status(shuttle_id)
    pos = status.feedback_position_si
    return pos[0], pos[1]


def find_closest_neighbor(current_shuttle_id, shuttles):
    current_x, current_y = get_xy(current_shuttle_id)
    min_distance = float("inf")

    for other_id in shuttles:
        if other_id == current_shuttle_id:
            continue

        other_x, other_y = get_xy(other_id)
        distance = math.dist((current_x, current_y), (other_x, other_y))

        if distance < min_distance:
            min_distance = distance

    return min_distance

def row_to_y(row_index):
    return row_index * SEGMENT_SIZE + OFFSET


def tile_to_coord(tile_id):
    """
    Muuntaa tile-id:n oikeaksi koordinaatiksi kuvan layoutissa:

    1-8   = vasen erikoissarake
    9-28  = vasen pystykaista
    29-48 = oikea pystykaista
    49-56 = oikea erikoissarake
    """
    # Vasen erikoissarake
    if 1 <= tile_id <= 8:
        row = SPECIAL_ROWS[tile_id - 1]
        return (X_COL_1, row_to_y(row))

    # Vasen pystykaista
    elif 9 <= tile_id <= 28:
        row = tile_id - 9
        return (X_COL_2, row_to_y(row))

    # Oikea pystykaista
    elif 29 <= tile_id <= 48:
        row = tile_id - 29
        return (X_COL_3, row_to_y(row))

    # Oikea erikoissarake
    elif 49 <= tile_id <= 56:
        row = SPECIAL_ROWS[tile_id - 49]
        return (X_COL_4, row_to_y(row))

    else:
        raise ValueError(f"Unsupported tile_id: {tile_id}")

START_GOAL_8 = {
    1: 53,
    2: 54,
    3: 55,
    4: 56,
    5: 49,
    6: 50,
    7: 51,
    8: 52,
}

START_GOAL_16 = {
    1: 53, 2: 54, 3: 55, 4: 56,
    5: 49, 6: 50, 7: 51, 8: 52,
    49: 5, 50: 6, 51: 7, 52: 8,
    53: 1, 54: 2, 55: 3, 56: 4,
}


def get_start_goal_pairs(total_shuttles):
    if total_shuttles == 8:
        return START_GOAL_8
    elif total_shuttles == 16:
        return START_GOAL_16
    else:
        raise ValueError("Only 8 or 16 shuttles supported")


def assign_goals(shuttles, total_shuttles):
    mapping = get_start_goal_pairs(total_shuttles)
    start_tiles = list(mapping.keys())
    goals = {}
    start_info = {}

    for shuttle_id, start_tile in zip(shuttles, start_tiles):
        goal_tile = mapping[start_tile]

        start_info[shuttle_id] = {
            "start_tile": start_tile,
            "start_coord": tile_to_coord(start_tile)
        }

        goals[shuttle_id] = {
            "goal_tile": goal_tile,
            "goal_coord": tile_to_coord(goal_tile)
        }

        print(
            f"Shuttle {shuttle_id}: "
            f"start tile {start_tile} -> goal tile {goal_tile}, "
            f"goal coord {goals[shuttle_id]['goal_coord']}"
        )

    return start_info, goals

def distance_to_point(shuttle_id, point):
    x, y = get_xy(shuttle_id)
    return math.dist((x, y), point)

def get_directional_lane_x(start_coord, goal_coord):
    _, start_y = start_coord
    _, goal_y = goal_coord

    if goal_y > start_y:
        return X_COL_3
    elif goal_y < start_y:
        return X_COL_2
    else:
        start_x, _ = start_coord
        if abs(start_x - X_COL_2) <= abs(start_x - X_COL_3):
            return X_COL_2
        else:
            return X_COL_3

def move_to_point(shuttle_id, target, speed):
    target_x, target_y = target

    bot.linear_motion_si(
        cmd_label=CMD_LABEL,
        xbot_id=shuttle_id,
        position_mode=pm.POSITIONMODE.ABSOLUTE,
        path_type=pm.LINEARPATHTYPE.DIRECT,
        target_xmeters=target_x,
        target_ymeters=target_y,
        final_speed_meters_ps=0.0,
        max_speed_meters_ps=speed,
        max_acceleration_meters_ps2=MAX_ACCEL,
        corner_radius=0.0
    )

def build_route(start_coord, goal_coord):
    start_x, start_y = start_coord
    goal_x, goal_y = goal_coord

    lane_x = get_directional_lane_x(start_coord, goal_coord)

    wp1 = (lane_x, start_y)
    wp2 = (lane_x, goal_y)
    wp3 = (goal_x, goal_y)

    route = []

    # lisää waypoint vain jos siinä ei jo olla
    if math.dist((start_x, start_y), wp1) > GOAL_TOLERANCE:
        route.append(wp1)

    if math.dist(route[-1] if route else (start_x, start_y), wp2) > GOAL_TOLERANCE:
        route.append(wp2)

    if math.dist(route[-1] if route else (start_x, start_y), wp3) > GOAL_TOLERANCE:
        route.append(wp3)

    return route

def main():
    #Establish connection to Planar Motor Tool and Planar Motor Simulation Tool
    connection = ConnectToSim()

    xbot_ids = bot.get_xbot_ids()

    if xbot_ids.PmcRtn != pm.PMCRTN.ALLOK:
        print(f"Failed to get xbot IDs: {xbot_ids.PmcRtn}")
        return

    shuttle_ids = list(xbot_ids.xbot_ids_array[:xbot_ids.xbot_count])
    shuttles = shuttle_ids[:TOTAL_SHUTTLES]

    logic = BehaviorLogic()
    behaviors = logic.assign_behaviors(shuttles, PUSHER_RATIO, ASSIGNMENT_MODE)
    start_info, goals = assign_goals(shuttles, TOTAL_SHUTTLES)

    print("Shuttles:", shuttles)
    print("Behaviors:", behaviors)
    print("Goals:", goals)

    #Routes and phases
    routes = {}
    route_phase = {}
    command_sent_for_phase = {}

    for shuttle in shuttles:
        start_coord = start_info[shuttle]["start_coord"]
        goal_coord = goals[shuttle]["goal_coord"]

        routes[shuttle] = build_route(start_coord, goal_coord)
        route_phase[shuttle] = 0
        command_sent_for_phase[shuttle] = -1

    print("Shuttles:", shuttles)
    print("Behaviors:", behaviors)

    for shuttle in shuttles:
        print(f"Shuttle {shuttle} route: {routes[shuttle]}")

    while True:
        all_done = True
        for shuttle in shuttles:
            phase = route_phase[shuttle]

            if phase >= len(routes[shuttle]):
                continue

            all_done = False

            neighbor_distance = find_closest_neighbor(shuttle, shuttles)
            new_speed = logic.find_velocity(behaviors[shuttle],
                                            neighbor_distance)
            current_target = routes[shuttle][phase]

            # Jos ollaan jo waypointissa, siirrytään seuraavaan vaiheeseen
            if distance_to_point(shuttle, current_target) <= GOAL_TOLERANCE:
                route_phase[shuttle] += 1

                if route_phase[shuttle] >= len(routes[shuttle]):
                    print(f"Shuttle {shuttle} reached final goal.")
                    continue

                phase = route_phase[shuttle]
                current_target = routes[shuttle][phase]

            # Lähetä komento vain kerran per vaihe
            if command_sent_for_phase[shuttle] != phase:
                move_to_point(shuttle, current_target, new_speed)
                command_sent_for_phase[shuttle] = phase

            print(
                f"shuttle={shuttle}, "
                f"behavior={behaviors[shuttle]}, "
                f"phase={phase + 1}/{len(routes[shuttle])}, "
                f"dist={neighbor_distance:.3f}, "
                f"speed={new_speed:.3f}, "
                f"target=({current_target[0]:.3f}, {current_target[1]:.3f})"
            )

        if all_done:
            print("All shuttles reached their goals.")
            break

        time.sleep(LOOP_DELAY)


if __name__ == "__main__":
    main()