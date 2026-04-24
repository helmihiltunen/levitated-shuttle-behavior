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
import os
import csv
import numpy as np
import mss
import threading
import cv2

DEBUG = 1
CONFIG = 2
recording = True


#Parameters

STARTS_GOALS_1 = {
    2: 54,
    5: 50,
    29: 3,
    7: 53,
    28: 1,
    3: 52,
    6: 55,
    4: 8
}

STARTS_GOALS_2 = {
    1: 56,
    8: 49,
    3: 52,
    6: 53,
    2: 55,
    7: 50,
    4: 51,
    5: 54
}

if CONFIG == 1:
    STARTS_GOALS = STARTS_GOALS_1
else:
    STARTS_GOALS = STARTS_GOALS_2

TOTAL_SHUTTLES = len(STARTS_GOALS)
PUSHER_RATIO = 1.00
BASE_SPEED = 0.3

SHUTTLE_SIZE = 0.12
PASSIVE_SAFE_DISTANCE = 0.30 + SHUTTLE_SIZE
PASSIVE_CAUTION_DISTANCE = 0.24 + SHUTTLE_SIZE
PASSIVE_STOP_DISTANCE = 0.06 + SHUTTLE_SIZE

MIN_Y_GAP = int(SHUTTLE_SIZE * 1.0)

PUSHER_SAFE_DISTANCE = 0.48 + SHUTTLE_SIZE
PUSHER_CAUTION_DISTANCE = 0.12 + SHUTTLE_SIZE
PUSHER_STOP_DISTANCE = 0.12 + SHUTTLE_SIZE

WAYPOINT_BLOCK_DISTANCE = 0.10 + SHUTTLE_SIZE
SAME_LANE_THRESHOLD = 0.08

ASSIGNMENT_MODE = "random"   # "even" or "random"
MAX_ACCEL = 1.0
GOAL_TOLERANCE = 0.02
CMD_LABEL = 1

SEGMENT_SIZE = 0.24
OFFSET = 0.12


MIN_COL1_TILE_ID = 1
MAX_COL1_TILE_ID = 8
MIN_COL2_TILE_ID = 9
MAX_COL2_TILE_ID = 28
MIN_COL3_TILE_ID = 29
MAX_COL3_TILE_ID = 48
MIN_COL4_TILE_ID = 49
MAX_COL4_TILE_ID = 56


# X-coordinates for each column
X_COL_1 = OFFSET
X_COL_2 = OFFSET + SEGMENT_SIZE
X_COL_3 = OFFSET + 2 * SEGMENT_SIZE
X_COL_4 = OFFSET + 3 * SEGMENT_SIZE

# Start and goal rows
SPECIAL_ROWS = [0, 1, 6, 7, 12, 13, 18, 19]

def row_to_y(row_index):
    return row_index * SEGMENT_SIZE + OFFSET

#safety margins and parameters for solving deadlocks
CRITICAL_MARGIN = 0.12
CRITICAL_X_MIN = X_COL_2 - CRITICAL_MARGIN
CRITICAL_X_MAX = X_COL_3 + CRITICAL_MARGIN
CRITICAL_Y_MIN = row_to_y(6) - CRITICAL_MARGIN
CRITICAL_Y_MAX = row_to_y(13) + CRITICAL_MARGIN

DEADLOCK_DISTANCE_EPS = 0.02
DEADLOCK_TIME_SEC = 2.0
COLLISION_DISTANCE = SHUTTLE_SIZE

RESULTS_FILE = "simulation_results_notfinal.csv"

class ConnectToSim:
    #Establish connection to simulation
    def __init__(self):
        sys.auto_search_and_connect_to_pmc()
        sys.gain_mastership()

class BehaviorLogic:
    def assign_behaviors(self, shuttles, pusher_ratio, mode="even"):
        #assigning behavior logics for each shuttle
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
    #Finding movement speed based on behavior logic
        if behavior_type == "passive":
            if neighbor_distance < PASSIVE_STOP_DISTANCE:
                return 0.0
            elif neighbor_distance < PASSIVE_CAUTION_DISTANCE:
                return BASE_SPEED * 0.25
            elif neighbor_distance < PASSIVE_SAFE_DISTANCE:
                return BASE_SPEED * 0.5
            else:
                return BASE_SPEED

        elif behavior_type == "pusher":
            if neighbor_distance < PUSHER_STOP_DISTANCE:
                return 0.0
            elif neighbor_distance < PUSHER_CAUTION_DISTANCE:
                return BASE_SPEED * 0.90
            elif neighbor_distance < PUSHER_SAFE_DISTANCE:
                return BASE_SPEED * 1.40
            else:
                return BASE_SPEED * 1.15


def get_xy(shuttle_id):
    #Find xy position
    status = bot.get_xbot_status(shuttle_id)
    pos = status.feedback_position_si
    return pos[0], pos[1]


def distance_between_points(a, b):
    return math.dist(a, b)

def tile_to_coord(tile_id):
    #Find coordinates for the middle point of each tile
    if MIN_COL1_TILE_ID <= tile_id <= MAX_COL1_TILE_ID:
        row = SPECIAL_ROWS[tile_id - MIN_COL1_TILE_ID]
        return (X_COL_1, row_to_y(row))

    elif MIN_COL2_TILE_ID <= tile_id <= MAX_COL2_TILE_ID:
        row = tile_id - MIN_COL2_TILE_ID
        return (X_COL_2, row_to_y(row))

    elif MIN_COL3_TILE_ID <= tile_id <= MAX_COL3_TILE_ID:
        row = tile_id - MIN_COL3_TILE_ID
        return (X_COL_3, row_to_y(row))

    elif MIN_COL4_TILE_ID <= tile_id <= MAX_COL4_TILE_ID:
        row = SPECIAL_ROWS[tile_id - MIN_COL4_TILE_ID]
        return (X_COL_4, row_to_y(row))

    else:
        raise ValueError(f"Unsupported tile_id: {tile_id}")



def get_start_goal_pairs(total_shuttles):
    #Choosing right start and goal points for the shuttle count
    if total_shuttles == 2:
        return START_GOAL_2_TEST
    elif total_shuttles == 4:
        return START_GOAL_4_TEST
    elif total_shuttles == 8:
        return START_GOAL_8
    elif total_shuttles == 16:
        return STARTS_GOALS
    else:
        raise ValueError("Only 2, 4, 8 or 16 shuttles supported")


def assign_goals(shuttles, total_shuttles):
    #assigning goal coordinates for current shuttle
    # mapping = get_start_goal_pairs(total_shuttles)
    mapping = STARTS_GOALS
    start_tiles = list(mapping.keys())
    start_tile_coords = {tile: tile_to_coord(tile) for tile in start_tiles}

    start_info = {}
    goals = {}

    candidates = []

    # Trying to find the shuttle closest to the start
    for shuttle_id in shuttles:
        sx, sy = get_xy(shuttle_id)
        for start_tile, coord in start_tile_coords.items():
            dist = math.dist((sx, sy), coord)
            candidates.append((dist, shuttle_id, start_tile))

    candidates.sort(key=lambda x: x[0])

    assigned_shuttles = set()
    assigned_tiles = set()

    for dist, shuttle_id, start_tile in candidates:
        if shuttle_id in assigned_shuttles:
            continue
        if start_tile in assigned_tiles:
            continue

        goal_tile = mapping[start_tile]

        start_info[shuttle_id] = {
            "start_tile": start_tile,
            "start_coord": tile_to_coord(start_tile)
        }

        goals[shuttle_id] = {
            "goal_tile": goal_tile,
            "goal_coord": tile_to_coord(goal_tile)
        }

        assigned_shuttles.add(shuttle_id)
        assigned_tiles.add(start_tile)


    return start_info, goals


def distance_to_point(shuttle_id, point):
    #Calculate distance to given point
    x, y = get_xy(shuttle_id)
    return math.dist((x, y), point)


def get_directional_lane_x(start_coord, goal_coord):
    _, start_y = start_coord
    _, goal_y = goal_coord

    if goal_y < start_y:
        return X_COL_2

    if goal_y > start_y:
        return X_COL_3

    start_x, _ = start_coord
    if abs(start_x - X_COL_2) <= abs(start_x - X_COL_3):
        return X_COL_2
    return X_COL_3


def move_to_point(shuttle_id, target, speed):
    target_x, target_y = target
    #Move command to next waypoint
    if speed > 0:
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
    else: bot.stop_motion(shuttle_id)


def build_route(start_coord, goal_coord):
    #Calculate waypoints for the shuttles
    start_x, start_y = start_coord
    goal_x, goal_y = goal_coord

    lane_x = get_directional_lane_x(start_coord, goal_coord)

    wp1 = (lane_x, start_y)
    wp2 = (lane_x, goal_y)
    wp3 = (goal_x, goal_y)

    route = [wp1, wp2, wp3]


    return route

def current_waypoint_free(shuttle_id, routes, route_phase, shuttles):
    phase = route_phase[shuttle_id]
    if phase >= len(routes[shuttle_id]):
        return True

    current_target = routes[shuttle_id][phase]
    return not point_is_occupied(current_target, shuttle_id, shuttles)

def advance_phase_if_needed(shuttle_id, routes, route_phase):
    #Assigning new waypoint if shuttle has reached previous one
    while route_phase[shuttle_id] < len(routes[shuttle_id]):
        current_target = routes[shuttle_id][route_phase[shuttle_id]]
        #current_target = routes[shuttle_id][0]

        if distance_to_point(shuttle_id, current_target) <= GOAL_TOLERANCE:
           route_phase[shuttle_id] += 1
        else:
            break


def find_closest_forward_neighbor(current_shuttle_id, shuttles, current_target):
    #Calculate distance to the closest shuttle in front of current shuttle
    current_x, current_y = get_xy(current_shuttle_id)
    target_x, target_y = current_target

    move_dx = target_x - current_x
    move_dy = target_y - current_y
    move_len = math.hypot(move_dx, move_dy)

    if move_len < 1e-9:
        return float("inf")

    dir_x = move_dx / move_len
    dir_y = move_dy / move_len

    min_forward_distance = float("inf")

    for other_id in shuttles:
        if other_id == current_shuttle_id:
            continue

        other_x, other_y = get_xy(other_id)

        rel_x = other_x - current_x
        rel_y = other_y - current_y

        forward_projection = rel_x * dir_x + rel_y * dir_y
        if forward_projection <= 0:
            continue

        lateral_distance = abs(rel_x * dir_y - rel_y * dir_x)

        if lateral_distance > SAME_LANE_THRESHOLD:
            continue

        distance = math.hypot(rel_x, rel_y)

        if distance < min_forward_distance:
            min_forward_distance = distance

    return min_forward_distance


def point_is_occupied(point, current_shuttle_id, shuttles, threshold=WAYPOINT_BLOCK_DISTANCE):
    #Check if another shuttle is too close to a waypoint
    for other_id in shuttles:
        if other_id == current_shuttle_id:
            continue

        other_pos = get_xy(other_id)
        if distance_between_points(point, other_pos) < threshold:
            return True

    return False

def is_in_critical_zone(point):
    x, y = point
    return CRITICAL_X_MIN <= x <= CRITICAL_X_MAX and CRITICAL_Y_MIN <= y <= CRITICAL_Y_MAX


def shuttle_in_critical_zone(shuttle_id):
    return is_in_critical_zone(get_xy(shuttle_id))


def critical_zone_holder(shuttles):
    for shuttle_id in shuttles:
        if shuttle_in_critical_zone(shuttle_id):
            return shuttle_id
    return None

def point_to_segment_distance(px, py, ax, ay, bx, by):
    abx = bx - ax
    aby = by - ay
    apx = px - ax
    apy = py - ay

    ab_len_sq = abx * abx + aby * aby
    if ab_len_sq < 1e-12:
        return math.hypot(px - ax, py - ay)

    t = (apx * abx + apy * aby) / ab_len_sq
    t = max(0.0, min(1.0, t))

    closest_x = ax + t * abx
    closest_y = ay + t * aby

    return math.hypot(px - closest_x, py - closest_y)

def path_is_clear(shuttle_id, target, shuttles, clearance):
    sx, sy = get_xy(shuttle_id)
    tx, ty = target

    for other_id in shuttles:
        if other_id == shuttle_id:
            continue

        ox, oy = get_xy(other_id)

        dist_to_path = point_to_segment_distance(ox, oy, sx, sy, tx, ty)
        if dist_to_path < clearance:
            return False

    return True

def is_vertical_move(shuttle_id, target):
    sx, sy = get_xy(shuttle_id)
    tx, ty = target
    return abs(ty - sy) > abs(tx - sx)

def vertical_lane_clearance_ok(shuttle_id, shuttles, current_target, min_spacing):
    current_x, current_y = get_xy(shuttle_id)
    target_x, target_y = current_target

    move_dy = target_y - current_y
    if abs(move_dy) < 1e-9:
        return True

    moving_up = move_dy > 0

    for other_id in shuttles:
        if other_id == shuttle_id:
            continue

        other_x, other_y = get_xy(other_id)

        if abs(other_x - current_x) > SAME_LANE_THRESHOLD:
            continue

        rel_y = other_y - current_y

        if moving_up and rel_y <= 0:
            continue
        if not moving_up and rel_y >= 0:
            continue

        if abs(rel_y) < min_spacing:
            return False

    return True

def next_waypoint_free(shuttle_id, routes, route_phase, shuttles):
    phase = route_phase[shuttle_id]

    if phase + 1 >= len(routes[shuttle_id]):
        return True

    next_target = routes[shuttle_id][phase + 1]
    return not point_is_occupied(next_target, shuttle_id, shuttles)


def allowed_to_move(shuttle_id, shuttles, routes, route_phase, current_target):
    # if point_is_occupied(current_target, shuttle_id, shuttles):


    if is_vertical_move(shuttle_id, current_target):
        if not vertical_lane_clearance_ok(
            shuttle_id,
            shuttles,
            current_target,
            min_spacing=MIN_Y_GAP,
            # min_spacing=PASSIVE_STOP_DISTANCE
        ):
            if shuttle_id == DEBUG:
                print('vertical lane clearance ok')
            return False
    else:
        if not path_is_clear(
            shuttle_id,
            current_target,
            shuttles,
            clearance=0.08
        ) and False:
            if shuttle_id == DEBUG:
                print('path is clear')

            return False
    return True

def path_enters_critical_zone(shuttle_id, target):
    sx, sy = get_xy(shuttle_id)
    tx, ty = target

    samples = 10
    for i in range(samples + 1):
        t = i / samples
        x = sx + t * (tx - sx)
        y = sy + t * (ty - sy)
        if is_in_critical_zone((x, y)):
            return True
    return False

def all_stuck(shuttles, previous_positions):
    moved = False

    for shuttle_id in shuttles:
        old_pos = previous_positions[shuttle_id]
        new_pos = get_xy(shuttle_id)
        if math.dist(old_pos, new_pos) > DEADLOCK_DISTANCE_EPS:
            moved = True
            break

    return not moved

def choose_yielding_shuttle(active_shuttles, behaviors, routes, route_phase):
    passive_shuttles = [s for s in active_shuttles if behaviors[s] == "passive"]
    pusher_shuttles = [s for s in active_shuttles if behaviors[s] == "pusher"]

    # passive väistää aina ennen pusheria
    candidate_group = passive_shuttles if passive_shuttles else pusher_shuttles

    # saman ryhmän sisällä väistää se, joka on kauimpana targetista
    return max(
        candidate_group,
        key=lambda s: distance_to_point(s, routes[s][route_phase[s]])
    )

def choose_speed(shuttle_id, shuttles, current_target, behavior_type, logic, routes, route_phase):
    #choosing speed based on whether next waypoint is occupied
    # and the distance to the closest neighbor in front of current shuttle
    neighbor_distance = find_closest_forward_neighbor(
        shuttle_id,
        shuttles,
        current_target
    )

    speed = logic.find_velocity(behavior_type, neighbor_distance)

    return speed, neighbor_distance

def write_results_to_csv(filename, row):
    file_exists = os.path.isfile(filename)

    with open(filename, "a", newline="", encoding="utf-8-sig") as f:
        writer = csv.writer(f, delimiter=";")

        if not file_exists:
            writer.writerow([
                "config",
                "total_bots",
                "pusher_ratio",
                "pushers",
                "loops_until_end",
                "unique_collisions",
                "stuck_shuttles"
            ])

        writer.writerow(row)

def count_pushers(behaviors):
    return sum(1 for behavior in behaviors.values() if behavior == "pusher")


def get_collision_pairs(shuttles, collision_distance=COLLISION_DISTANCE):
    collision_pairs = set()

    for i in range(len(shuttles)):
        for j in range(i + 1, len(shuttles)):
            shuttle_a = shuttles[i]
            shuttle_b = shuttles[j]

            pos_a = get_xy(shuttle_a)
            pos_b = get_xy(shuttle_b)

            if distance_between_points(pos_a, pos_b) < collision_distance:
                collision_pairs.add(tuple(sorted((shuttle_a, shuttle_b))))

    return collision_pairs

def screen_record(monitor):
    global recording

    output_filename = "partial_recording.mp4"
    fps = 20.0

    with mss.mss() as sct:
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        out = cv2.VideoWriter(
            output_filename,
            fourcc,
            fps,
            (monitor["width"], monitor["height"])
        )

        try:
            while recording:
                img = np.array(sct.grab(monitor))
                frame = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
                out.write(frame)
        finally:
            out.release()
            print("Video saved properly")


def main():
    global recording

    monitor = {'top': 95, 'left': 1078, 'width': 117, 'height': 520}

    recorder_thread = threading.Thread(
        target=screen_record,
        args=(monitor,),
        daemon=True
    )
    recorder_thread.start()

    try:
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

            routes = {}
            route_phase = {}
            finished_shuttles = set()
            stuck_shuttles = set()
            previous_positions = {}
            stuck_start_time = None

            loop_count = 0
            collision_pairs_seen = set()

            for shuttle in shuttles:
                if shuttle in finished_shuttles:
                    continue
                actual_start_coord = get_xy(shuttle)
                goal_coord = goals[shuttle]["goal_coord"]

                routes[shuttle] = build_route(actual_start_coord, goal_coord)
                route_phase[shuttle] = 0

                lane_x = get_directional_lane_x(actual_start_coord, goal_coord)
                print(
                    f"Shuttle {shuttle}: "
                    f"actual_start={actual_start_coord}, goal={goal_coord}, "
                    f"lane_x={lane_x}, route={routes[shuttle]}")

            print("Shuttles:", shuttles)
            print("Behaviors:", behaviors)

            for shuttle in shuttles:
                previous_positions[shuttle] = get_xy(shuttle)

            while True:
                loop_count += 1
                all_done = True

                targets = {shuttle: get_xy(shuttle) for shuttle in shuttles}
                speeds = {shuttle: 0 for shuttle in shuttles}

                for shuttle in shuttles:
                    #1. next phase if needed
                    advance_phase_if_needed(shuttle, routes, route_phase)
                    if shuttle == DEBUG: print(f'route {routes[shuttle]} route_phase {route_phase[shuttle]}')
                    # 2. if shuttle is in goal point add it to finished shuttles
                    # move to next shuttle
                    if route_phase[shuttle] >= len(routes[shuttle]):
                        finished_shuttles.add(shuttle)
                        move_to_point(shuttle, get_xy(shuttle), 0.01)
                        continue

                    all_done = False

                    # 3. get new current target
                    phase = route_phase[shuttle]
                    current_target = routes[shuttle][phase]
                    if shuttle == DEBUG: print(f'current target {current_target}')
                    # 4. find speed based on traffic
                    new_speed, neighbor_distance = choose_speed(
                        shuttle_id=shuttle,
                        shuttles=shuttles,
                        current_target=current_target,
                        behavior_type=behaviors[shuttle],
                        logic=logic,
                        routes=routes,
                        route_phase=route_phase
                    )

                    # 5. send new movement command if it isn't zero and shuttle is
                    # allowed to move
                    can_move = allowed_to_move(shuttle, shuttles, routes, route_phase, current_target)
                    if shuttle == DEBUG: print(f'can move {can_move}')


                    if can_move and new_speed > 1e-6:
                        targets[shuttle] = current_target
                        speeds[shuttle] = new_speed


                    dist_text = "inf" if neighbor_distance == float("inf") else f"{neighbor_distance:.3f}"
                    occupied_text = point_is_occupied(current_target, shuttle, shuttles)

                    if shuttle == DEBUG:
                        print(
                            f"shuttle={shuttle}, "
                            f"behavior={behaviors[shuttle]}, "
                            f"phase={phase + 1}/{len(routes[shuttle])}, "
                            f"forward_dist={dist_text}, "
                            f"occupied={occupied_text}, "
                            f"speed={new_speed:.3f}, "
                            f"target=({current_target[0]:.3f}, {current_target[1]:.3f})"
                        )
                for shuttle in shuttles:
                    move_to_point(shuttle, targets[shuttle], speeds[shuttle])

                current_collision_pairs = get_collision_pairs(shuttles)
                collision_pairs_seen.update(current_collision_pairs)

                if all_done:
                    print("All shuttles reached their goals.")

                    result_row = [
                        len(shuttles),
                        PUSHER_RATIO,
                        count_pushers(behaviors),
                        loop_count,
                        len(collision_pairs_seen),
                        len(stuck_shuttles)
                    ]

                    write_results_to_csv(RESULTS_FILE, result_row)
                    break

                active_shuttles = [s for s in shuttles if s not in finished_shuttles]

                if active_shuttles:
                    if all_stuck(active_shuttles, previous_positions):
                        if stuck_start_time is None:
                            stuck_start_time = time.time()
                        elif time.time() - stuck_start_time >= DEADLOCK_TIME_SEC:
                            stuck_shuttles = set(active_shuttles)

                            print("Simulation stopped: no shuttle moved within the deadlock time limit.")

                            result_row = [
                                CONFIG,
                                len(shuttles),
                                PUSHER_RATIO,
                                count_pushers(behaviors),
                                loop_count,
                                len(collision_pairs_seen),
                                len(stuck_shuttles)
                            ]

                            write_results_to_csv(RESULTS_FILE, result_row)
                            break
                    else:
                        stuck_start_time = None

                for shuttle in shuttles:
                    previous_positions[shuttle] = get_xy(shuttle)

    finally:
        recording = False
        recorder_thread.join(timeout=2)

if __name__ == "__main__":
    main()