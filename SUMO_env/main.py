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

from ortools.linear_solver import pywraplp
from sumolib import checkBinary
import traci
import traceback

import config as cfg
import csv
import json
from gen_route import generate_routefile_with_src_dst, generate_routefile_with_src_dst_event


# For debug
#from playsound import playsound

from IntersectionManager import IntersectionManager
#from myGraphic import Gui
#import myGraphic


#myGraphic.gui = Gui()

###################

vehNr = 0

def run():
    """execute the TraCI control loop"""
    simu_step = 0

    # Create a list with intersection managers
    intersection_manager_dict = dict()
    for idx in range(1, cfg.INTER_SIZE+1):
        for jdx in range(1, cfg.INTER_SIZE+1):
            intersection_manager_id = "%3.3o"%(idx) + "_" + "%3.3o"%(jdx)
            intersection_manager = IntersectionManager(intersection_manager_id)
            intersection_manager_dict[(idx, jdx)] = intersection_manager

    for idx in range(1, cfg.INTER_SIZE+1):
        for jdx in range(1, cfg.INTER_SIZE+1):
            if idx <= cfg.INTER_SIZE-1:
                intersection_manager_dict[(idx, jdx)].connect(1, intersection_manager_dict[(idx+1, jdx)], 3)

            if jdx <= cfg.INTER_SIZE-1:
                intersection_manager_dict[(idx, jdx)].connect(2, intersection_manager_dict[(idx, jdx+1)], 0)


    # Start simulation
    try:
        while traci.simulation.getMinExpectedNumber() > 0:

            if (simu_step*10)//1/10.0 == cfg.N_TIME_STEP:
                break

            traci.simulationStep()
            all_c = traci.vehicle.getIDList()

            # Update the position of each car
            for car_id in all_c:

                lane_id = traci.vehicle.getLaneID(car_id)

                is_handled = False
                for intersection_manager in intersection_manager_dict.values():
                    if intersection_manager.check_in_my_region(lane_id):

                        # TODO: get turning
                        car_turn = random.choice(["S", "L", "R"])

                        is_handled = True
                        intersection_manager.update_car(car_id, lane_id, simu_step, car_turn)
                        break
                if not is_handled:
                    # Leaving intersections
                    traci.vehicle.setSpeed(car_id, cfg.MAX_SPEED)

            for intersection_manager in intersection_manager_dict.values():
                intersection_manager.run(simu_step)


            simu_step += cfg.TIME_STEP



    except Exception as e:
        traceback.print_exc()


    #debug_t = threading.Thread(target=debug_ring)
    #debug_t.start()
    print(sys.argv[1], int(sys.argv[2]), int(sys.argv[3]), sys.argv[4])

    # Print out the measurements
    #print("Average total delay: ", total_delays/car_num)
    #print("Average delay by scheduling: ", total_delays_by_sche/car_num)
    print(intersection_manager.total_delays/intersection_manager.car_num, intersection_manager.total_delays_by_sche/intersection_manager.car_num, intersection_manager.car_num)

    print("avg_fuel = ",intersection_manager.total_fuel_consumption/intersection_manager.fuel_consumption_count)


    file_name2 = 'result/result.csv'

    with open(file_name2, 'a', newline='') as csvfile2:
        writer2 = csv.writer(csvfile2, dialect='excel-tab', quoting=csv.QUOTE_MINIMAL, delimiter = ',')
        to_write2 = [sys.argv[1], sys.argv[2], sys.argv[3],
                    sys.argv[4], "_", simu_step,
                    intersections[0].car_num,
                    intersections[1].car_num,
                    vehNr,
                    vehNr/6
                    ]
        writer2.writerow(to_write2)

    sys.stdout.flush()

    traci.close()



###########################
# Main function
if __name__ == "__main__":
    print("Usage: python code.py <arrival_rate (0~1.0)> <seed> <schedular> <grid_size n (nxn)>")
    sys.argv[4]

    seed = int(sys.argv[2])
    random.seed(seed)  # make tests reproducible
    numpy.random.seed(seed)
    arrival_rate = float(sys.argv[1])
    cfg.INTER_SIZE = int(sys.argv[4])

    sumoBinary = checkBinary('sumo-gui')

    # 0. Generate the intersection information files
    os.system("bash gen_intersection/gen_data.sh " + str(cfg.LANE_NUM_PER_DIRECTION))
    os.system("rm data/routes/*")

    # 1. Generate the route file for this simulation
    arrival_rate = float(sys.argv[1])
    vehNr = generate_routefile_with_src_dst(cfg.INTER_SIZE, arrival_rate, seed, cfg.N_TIME_STEP)

    # Load from the file
    src_dst_file_name = "%i_%s_%i_src_dst.json" % (cfg.INTER_SIZE, arrival_rate, seed)
    with open('data/routes/'+src_dst_file_name) as json_file:
        src_dst_dict = json.load(json_file)



    try:
        # 3. This is the normal way of using traci. sumo is started as a subprocess and then the python script connects and runs

        net_name = "lane%iby%i.net.xml" % (cfg.INTER_SIZE, cfg.INTER_SIZE)
        route_name = "%i_%s_%i.rou.xml" % (cfg.INTER_SIZE, arrival_rate, seed)
        traci.start([sumoBinary, "-c", "data/UDTA.sumocfg",
                                 "--step-length", str(cfg.TIME_STEP),
                                 "--collision.mingap-factor", "0",
                                 "-n", "data/net/" + net_name,
                                 "-r", "data/routes/" + route_name])

        # 4. Start running SUMO
        run()
    except Exception as e:
        traceback.print_exc()
