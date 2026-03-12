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

class ConnectToSim:
    def __init__(self):
        #initializing and connecting to simulation in pmst
        self.system = sys.SystemCommands()
        self.system.connect("127.0.0.1")
        self.system.enable_system()


def main():
    #Establish connection to Planar Motor Tool and Planar Motor Simulation Tool
    connection = ConnectToSim()
    # Finding shuttles and giving each of them id
    xbots = bot.XbotCommands()
    shuttles = xbots.get_xbots()
    for shuttle in shuttles:
        shuttle_id = shuttle.id

    #Resting time before next loop 0.1s
    time.sleep(0.1)

if __name__ == "__main__":
    main()