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

    # Initial variables
    seed = int(sys.argv[2])
    random.seed(seed)  # make tests reproducible
    numpy.random.seed(seed)
    arrival_rate = float(sys.argv[1])
    cfg.INTER_SIZE = int(sys.argv[4])

    # Initial SUMO
    #sumoBinary = checkBinary('sumo-gui')
    sumo = "sumo-gui "

    # 0. Generate the intersection information files
    os.system("bash gen_intersection/gen_data.sh " + str(cfg.LANE_NUM_PER_DIRECTION))
    os.system("rm data/routes/*")

    # 1. Generate the route file for this simulation
    vehNr = generate_routefile_with_src_dst(cfg.INTER_SIZE, arrival_rate, seed, cfg.N_TIME_STEP)

    try:
        # 3. Start TraCi
        net_name = "lane%iby%i.net.xml" % (cfg.INTER_SIZE, cfg.INTER_SIZE)
        route_name = "%i_%s_%i.rou.xml" % (cfg.INTER_SIZE, arrival_rate, seed)

        os.system(sumo + "-c data/UDTA.sumocfg --step-length " + str(cfg.TIME_STEP)
                    + " --collision.mingap-factor 0 -n data/net/"+net_name
                    + " -r data/routes/"+route_name + " --remote-port " + str(PORT))

    except Exception as e:
        traceback.print_exc()
