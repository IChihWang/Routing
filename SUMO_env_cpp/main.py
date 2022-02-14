from __future__ import absolute_import
from __future__ import print_function


import os
import sys

sys.path.append('/usr/share/sumo/tools/')
# we need to import python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import optparse
import random
import numpy
import threading
import time
import numpy as np

from sumolib import checkBinary
import traci
import traceback

import config as cfg
import csv
import json
from gen_route import generate_routefile_with_src_dst

HOST, PORT = "localhost", 9997

#  Start SUMO and listen to the port for TraCi commands

###########################
# Main function
if __name__ == "__main__":

    print("python main.py <arrival_rate> <seed> <INTER_SIZE> <TOP_N> <CHOOSE_CAR_OPTION> <Thread_num> <Iteration_num> <_CAR_TIME_ERROR> <_MEC_num_per_edge> <_Load_Balance T/F>")

    # Initial variables
    seed = int(sys.argv[2])
    random.seed(seed)  # make tests reproducible
    numpy.random.seed(seed)
    arrival_rate = float(sys.argv[1])
    cfg.INTER_SIZE = int(sys.argv[3])

    # Initial SUMO
    #sumoBinary = checkBinary('sumo-gui')
    sumo = "sumo "

    # 0. Generate the intersection information files
    os.system("bash gen_intersection/gen_data.sh " + str(cfg.LANE_NUM_PER_DIRECTION))
    os.system("rm data/routes/*")

    # 1. Generate the route file for this simulation
    vehNr = generate_routefile_with_src_dst(cfg.INTER_SIZE, arrival_rate, seed, cfg.N_TIME_STEP)

    try:
        # 3. Start TraCi
        net_name = "lane%iby%i.net.xml" % (cfg.INTER_SIZE, cfg.INTER_SIZE)
        route_name = "%i_%s_%i.rou.xml" % (cfg.INTER_SIZE, arrival_rate, seed)
        #background = "START /B "

        os.system(sumo + "-c data/UDTA.sumocfg --step-length " + str(cfg.TIME_STEP)
                    + " --collision.mingap-factor 0 -n data/net/"+net_name
                    + " -r data/routes/"+route_name + " --remote-port " + str(PORT)
                    + " --collision.action none &")

        #'''
        cpp_cmd = './x64/Release/main '
        cpp_cmd += str(cfg.INTER_SIZE) + " "
        cpp_cmd += "%i_%s_%i_src_dst.json " % (cfg.INTER_SIZE, arrival_rate, seed)
        cpp_cmd += str(cfg.N_TIME_STEP) + " "
        cpp_cmd += str(cfg.TIME_STEP) + " "
        cpp_cmd += sys.argv[4] + " "    # TOP_N
        cpp_cmd += sys.argv[5] + " "    # Choose car option
        cpp_cmd += sys.argv[6] + " "    # Thread number
        cpp_cmd += sys.argv[7] + " "    # Iteration num
        cpp_cmd += sys.argv[8] + " "    # _CAR_TIME_ERROR
        cpp_cmd += sys.argv[9] + " "    # _MEC_num_per_edge
        cpp_cmd += sys.argv[10] + " "    # _Load_Balance
        cpp_cmd += str(arrival_rate) + " "
        cpp_cmd += str(seed) + " "
        cpp_cmd += " &"
        os.system(cpp_cmd)
        #'''

    except Exception as e:
        traceback.print_exc()
