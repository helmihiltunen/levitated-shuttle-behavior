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

TOTAL_SHUTTLES = 12
PUSHER_RATIO = 0.25
LOOP_DELAY = 0.1
BASE_SPEED = 0.2
SAFE_DISTANCE = 0.20
CAUTION_DISTANCE = 0.10
ASSIGNMENT_MODE = "even"   # "even" or "random"

class ConnectToSim:
    def __init__(self):
        #initializing and connecting to simulation in pmst
        self.system = sys.SystemCommands()
        self.system.connect("127.0.0.1")
        self.system.enable_system()

class BehaviorLogic:
    def assign_behaviors(self, shuttles, pusher_ratio, mode="even"):
        behaviors = {}
        n = len(shuttles)
        pusher_count = int(n * pusher_ratio)

        if pusher_count <= 0:
            for shuttle in shuttles:
                behaviors[shuttle.id] = "passive"
            return behaviors

        if mode == "random":
            pusher_ids = set(shuttle.id for shuttle in random.sample(shuttles, pusher_count))

        elif mode == "even":
            indices = []
            if pusher_count == 1:
                indices = [0]
            else:
                step = (n - 1) / (pusher_count - 1)
                indices = [round(i * step) for i in range(pusher_count)]

            pusher_ids = set(shuttles[i].id for i in indices)

        else:
            raise ValueError("mode must be 'even' or 'random'")

        for shuttle in shuttles:
            if shuttle.id in pusher_ids:
                behaviors[shuttle.id] = "pusher"
            else:
                behaviors[shuttle.id] = "passive"

        return behaviors

    def find_velocity(self, behavior_type, neighbor_distance):
        if behavior_type == "passive":
            if neighbor_distance < CAUTION_DISTANCE:
                new_speed = BASE_SPEED * 0.25

            elif neighbor_distance < SAFE_DISTANCE:
                new_speed = BASE_SPEED * 0.5

            else:
                new_speed = BASE_SPEED

        if behavior_type == "pusher":
            if neighbor_distance < CAUTION_DISTANCE:
                new_speed = BASE_SPEED * 0.75

            elif neighbor_distance < SAFE_DISTANCE:
                new_speed = BASE_SPEED * 1.2

            else:
                new_speed = BASE_SPEED

        return new_speed

def main():
    #Establish connection to Planar Motor Tool and Planar Motor Simulation Tool
    connection = ConnectToSim()
    # Finding shuttles and giving each of them id
    xbots = bot.XbotCommands()
    shuttles = xbots.get_xbots()

    shuttles = shuttles[:TOTAL_SHUTTLES]

    logic = BehaviorLogic()
    behaviors = logic.assign_behaviors(shuttles, PUSHER_RATIO, ASSIGNMENT_MODE)

    while True:
        for shuttle in shuttles:
            pos = shuttle.get_position()
            speed = shuttle.get_velocity()

            neighbor_distance = 0.2 #temporary test value
            new_speed = logic.find_velocity(behaviors[shuttle.id], neighbor_distance)

            shuttle.set_velocity(new_speed)

        #Resting time before next loop 0.1s
        time.sleep(LOOP_DELAY)

if __name__ == "__main__":
    main()