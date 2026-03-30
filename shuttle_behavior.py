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
SEGMENT_SIZE = 0.24         # meters
OFFSET = 0.12               # meters

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


def tile_to_coord(tile_id, columns=8):
#changes tile numbers to coordinates for the simulation
    row = (tile_id - 1) // columns
    col = (tile_id - 1) % columns

    x = col * SEGMENT_SIZE + OFFSET
    y = row * SEGMENT_SIZE + OFFSET

    return (x, y)

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

    for shuttle_id, start_tile in zip(shuttles, start_tiles):
        goal_tile = mapping[start_tile]
        goals[shuttle_id] = tile_to_coord(goal_tile)

        print(
            f"Shuttle {shuttle_id}: "
            f"start tile {start_tile} -> goal tile {goal_tile}, "
            f"goal coord {goals[shuttle_id]}"
        )

    return goals

def distance_to_goal(shuttle_id, goals):
    x, y = get_xy(shuttle_id)
    goal_x, goal_y = goals[shuttle_id]
    return math.dist((x, y), (goal_x, goal_y))

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
    goals = assign_goals(shuttles, TOTAL_SHUTTLES)

    print("Shuttles:", shuttles)
    print("Behaviors:", behaviors)
    print("Goals:", goals)

    all_goal_x = [goals[shuttle][0] for shuttle in shuttles]
    all_goal_y = [goals[shuttle][1] for shuttle in shuttles]

    while True:
        for shuttle in shuttles:
            neighbor_distance = find_closest_neighbor(shuttle, shuttles)
            new_speed = logic.find_velocity(behaviors[shuttle],
                                            neighbor_distance)

            goal_x, goal_y = goals[shuttle]

            if distance_to_goal(shuttle, goals) > GOAL_TOLERANCE:
                bot.linear_motion_si(
                    cmd_label=CMD_LABEL,
                    xbot_id=shuttle,
                    position_mode=pm.POSITIONMODE.ABSOLUTE,
                    path_type=pm.LINEARPATHTYPE.DIRECT,
                    target_xmeters=goal_x,
                    target_ymeters=goal_y,
                    final_speed_meters_ps=0.0,
                    max_speed_meters_ps=new_speed,
                    max_acceleration_meters_ps2=MAX_ACCEL,
                    corner_radius=0.0
                )

            print(
                f"shuttle={shuttle}, "
                f"behavior={behaviors[shuttle]}, "
                f"dist={neighbor_distance:.3f}, "
                f"speed={new_speed:.3f}, "
                f"goal=({goal_x:.3f}, {goal_y:.3f})"
            )

        time.sleep(LOOP_DELAY)

if __name__ == "__main__":
    main()