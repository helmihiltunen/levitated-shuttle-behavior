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

#Parameters
TOTAL_SHUTTLES = 2
PUSHER_RATIO = 0
#LOOP_DELAY = 0.01
BASE_SPEED = 0.2

SHUTTLE_SIZE = 0.12
PASSIVE_SAFE_DISTANCE = 0.30 + SHUTTLE_SIZE
PASSIVE_CAUTION_DISTANCE = 0.20 + SHUTTLE_SIZE
PASSIVE_STOP_DISTANCE = 0.15 + SHUTTLE_SIZE

PUSHER_SAFE_DISTANCE = 0.18 + SHUTTLE_SIZE
PUSHER_CAUTION_DISTANCE = 0.15 + SHUTTLE_SIZE
PUSHER_STOP_DISTANCE = 0.13 + SHUTTLE_SIZE

WAYPOINT_BLOCK_DISTANCE = 0.10 + SHUTTLE_SIZE
SAME_LANE_THRESHOLD = 0.08

ASSIGNMENT_MODE = "random"   # "even" or "random"
MAX_ACCEL = 1.0
GOAL_TOLERANCE = 0.02
CMD_LABEL = 1

SEGMENT_SIZE = 0.24
OFFSET = 0.12

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
        if neighbor_distance == float("inf"):
            return BASE_SPEED

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

        return BASE_SPEED

def get_xy(shuttle_id):
    #Find xy position
    status = bot.get_xbot_status(shuttle_id)
    pos = status.feedback_position_si
    return pos[0], pos[1]


def distance_between_points(a, b):
    return math.dist(a, b)

def tile_to_coord(tile_id):
    #Find coordinates for the middle point of each tile
    if 1 <= tile_id <= 8:
        row = SPECIAL_ROWS[tile_id - 1]
        return (X_COL_1, row_to_y(row))

    elif 9 <= tile_id <= 28:
        row = tile_id - 9
        return (X_COL_2, row_to_y(row))

    elif 29 <= tile_id <= 48:
        row = tile_id - 29
        return (X_COL_3, row_to_y(row))

    elif 49 <= tile_id <= 56:
        row = SPECIAL_ROWS[tile_id - 49]
        return (X_COL_4, row_to_y(row))

    else:
        raise ValueError(f"Unsupported tile_id: {tile_id}")

# Start tile number: Goal tile number for 8 and 16 bots
START_GOAL_2 = {
    1: 53,
    49: 5,
}
START_GOAL_4 = {
    1: 53,
    2: 54,
    49: 5,
    50: 6,
}
START_GOAL_8 = {
    1: 53,
    2: 54,
    3: 55,
    4: 56,
    49: 5,
    50: 6,
    51: 7,
    52: 8,
}

START_GOAL_16 = {
    1: 53, 2: 54, 3: 55, 4: 56,
    5: 49, 6: 50, 7: 51, 8: 52,
    49: 5, 50: 6, 51: 7, 52: 8,
    53: 1, 54: 2, 55: 3, 56: 4,
}


def get_start_goal_pairs(total_shuttles):
    #Choosing right start and goal points for the shuttle count
    if total_shuttles == 2:
        return START_GOAL_2
    elif total_shuttles == 4:
        return START_GOAL_4
    elif total_shuttles == 8:
        return START_GOAL_8
    elif total_shuttles == 16:
        return START_GOAL_16
    else:
        raise ValueError("Only 2, 4, 8 or 16 shuttles supported")


def assign_goals(shuttles, total_shuttles):
    #assigning goal coordinates for current shuttle
    mapping = get_start_goal_pairs(total_shuttles)

    start_tiles = list(mapping.keys())
    start_tile_coords = {tile: tile_to_coord(tile) for tile in start_tiles}

    start_info = {}
    goals = {}

    candidates = []
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
    #Calculate waypoints for the shuttles
    start_x, start_y = start_coord
    goal_x, goal_y = goal_coord

    lane_x = get_directional_lane_x(start_coord, goal_coord)

    wp1 = (lane_x, start_y)
    wp2 = (lane_x, goal_y)
    wp3 = (goal_x, goal_y)

    route = []

    if math.dist((start_x, start_y), wp1) > GOAL_TOLERANCE:
        route.append(wp1)

    if math.dist(route[-1] if route else (start_x, start_y), wp2) > GOAL_TOLERANCE:
        route.append(wp2)

    if math.dist(route[-1] if route else (start_x, start_y), wp3) > GOAL_TOLERANCE:
        route.append(wp3)

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



def next_waypoint_free(shuttle_id, routes, route_phase, shuttles):
    phase = route_phase[shuttle_id]

    if phase + 1 >= len(routes[shuttle_id]):
        return True

    next_target = routes[shuttle_id][phase + 1]
    return not point_is_occupied(next_target, shuttle_id, shuttles)


def allowed_to_move(shuttle_id, shuttles, routes, route_phase, current_target):
    if point_is_occupied(current_target, shuttle_id, shuttles):
        return False

    if not path_is_clear(
        shuttle_id,
        current_target,
        shuttles,
        clearance=SHUTTLE_SIZE + 0.06
    ):
        return False

    holder = critical_zone_holder(shuttles)
    if path_enters_critical_zone(shuttle_id, current_target):
        if holder is not None and holder != shuttle_id:
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

def main():
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
    previous_positions = {}
    stuck_start_time = None

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
        all_done = True

        for shuttle in shuttles:
            #1. next phase if needed
            advance_phase_if_needed(shuttle, routes, route_phase)

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

            if can_move and new_speed > 1e-6:
                move_to_point(shuttle, current_target, new_speed)
            else:
                move_to_point(shuttle, get_xy(shuttle), 0.01)

            dist_text = "inf" if neighbor_distance == float("inf") else f"{neighbor_distance:.3f}"
            occupied_text = point_is_occupied(current_target, shuttle, shuttles)

            print(
                f"shuttle={shuttle}, "
                f"behavior={behaviors[shuttle]}, "
                f"phase={phase + 1}/{len(routes[shuttle])}, "
                f"forward_dist={dist_text}, "
                f"occupied={occupied_text}, "
                f"speed={new_speed:.3f}, "
                f"target=({current_target[0]:.3f}, {current_target[1]:.3f})"
            )

        if all_done:
            print("All shuttles reached their goals.")
            break

        active_shuttles = [s for s in shuttles if s not in finished_shuttles]

        if active_shuttles:
            if all_stuck(active_shuttles, previous_positions):
                if stuck_start_time is None:
                    stuck_start_time = time.time()
                elif time.time() - stuck_start_time >= DEADLOCK_TIME_SEC:
                    yielding_shuttle = choose_yielding_shuttle(
                        active_shuttles,
                        behaviors,
                        routes,
                        route_phase
                    )
                    if route_phase[yielding_shuttle] < len(
                            routes[yielding_shuttle]):
                        current_target = routes[yielding_shuttle][
                            route_phase[yielding_shuttle]]
                        if distance_to_point(yielding_shuttle,
                                             current_target) > GOAL_TOLERANCE:
                            move_to_point(yielding_shuttle,
                                          get_xy(yielding_shuttle), 0.01)
                    stuck_start_time = None
            else:
                stuck_start_time = None

        for shuttle in shuttles:
            previous_positions[shuttle] = get_xy(shuttle)

        #time.sleep(LOOP_DELAY)


if __name__ == "__main__":
    main()