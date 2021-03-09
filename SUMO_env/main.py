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

import socket
import multiprocessing


# For debug
#from playsound import playsound

from IntersectionManager import IntersectionManager
#from myGraphic import Gui
#import myGraphic


# Initial the client
HOST, PORT = "localhost", 9996


def initial_server_handler(HOST, PORT):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((HOST, PORT))

    to_handler_queue = multiprocessing.Queue()
    from_handler_queue = multiprocessing.Queue()

    init_message = "My_grid_size:" + str(cfg.INTER_SIZE)
    init_message += ":My_schedule_period:" + "{:.2f}".format(cfg.SCHEDULING_PERIOD)
    init_message += ":My_routing_period_num:" + str(cfg.ROUTING_PERIOD_NUM)
    sock.send(init_message.encode())
    reply_message = sock.recv(1024).decode()
    print("Server replies: ", reply_message)

    handler_process = multiprocessing.Process(target=handler, args=(sock, to_handler_queue, from_handler_queue))
    handler_process.start()

    return (handler_process, to_handler_queue, from_handler_queue)

def handler(sock, to_handler_queue, from_handler_queue):

    try:
        is_continue = True

        while(is_continue):
            # Get data from SUMO part
            send_str = to_handler_queue.get()
            # ===========   Block   ================
            if send_str == "End Connection":
                is_continue = False
                break

            # Send request
            send_str = send_str + "@"
            sock.sendall(send_str.encode())

            # Receive the result
            data = ""
            while len(data) == 0 or data[-1] != "@":
                get_str = sock.recv(8192)
                if get_str == b'':
                    from_handler_queue.put("End Connection")
                    break
                data += get_str.decode()

            from_handler_queue.put(data[:-1])       # Remove the "@" at the end

    except Exception as e:
        traceback.print_exc()
        from_handler_queue.put("End Connection")

    sock.close()

def run_sumo(_handler_process, _to_handler_queue, _from_handler_queue, src_dst_dict):
    simu_step = 0

    handler_process = _handler_process
    to_handler_queue = _to_handler_queue
    from_handler_queue = _from_handler_queue

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
        # Record car info
        car_info = dict()

        while traci.simulation.getMinExpectedNumber() > 0:
            # Terminate the simulation
            if (simu_step*10)//1/10.0 == cfg.N_TIME_STEP:
                to_handler_queue.put("End Connection")
                break

            '''
            if (simu_step*10)//1%10 == 0:
                print(intersection_manager_dict[(1, 1)].ID)
                print(intersection_manager_dict[(1, 1)].my_road_info)
                print('---')
                print(intersection_manager_dict[(1, 1)].others_road_info)
                print(str(simu_step) + '==============')
            #'''

            traci.simulationStep()
            all_c = traci.vehicle.getIDList()

            if simu_step%cfg.ROUTING_PERIOD < cfg.TIME_STEP:
                server_send_str = ""
                for car_id, car in car_info.items():
                    intersection_manager = car_info[car_id]["intersection_manager"]
                    if intersection_manager != None:
                        car_data = intersection_manager.get_car_info_for_route(car_id)
                        if car_data != None:
                            position_at_offset = car_data[0]
                            time_offset_step = car_data[1]
                            src_intersection_id = car_data[2]
                            direction_of_src_intersection = car_data[3]
                            server_send_str += car_id + ","
                            server_send_str += car["route_state"] + ","
                            server_send_str += str(car["car_length"]) + ","
                            server_send_str += src_intersection_id + ","
                            server_send_str += str(direction_of_src_intersection) + ","
                            server_send_str += str(time_offset_step) + ","      # Step that the datacenter needs to take
                            server_send_str += "{:.2f}".format(position_at_offset) + ","    # The position at the specific time
                            server_send_str += str(car["dst_node_idx"]) + ";"
                        else:
                            server_send_str += car_id + ","
                            server_send_str += "Pause" + ";"
                    else:
                        server_send_str += car_id + ","
                        server_send_str += "Exit" + ";"
                to_handler_queue.put(server_send_str)

            if (simu_step+cfg.TIME_STEP)%cfg.ROUTING_PERIOD < cfg.TIME_STEP:
                route_data = from_handler_queue.get()

                if route_data == "End Connection":
                    break
                print(route_data)

            # Update the position of each car
            for car_id in all_c:

                lane_id = traci.vehicle.getLaneID(car_id)

                # Dummy: Generate routes (turnings)
                if not car_id in car_info:
                    car_info[car_id] = dict()
                    route = []

                    # TODO: change the route
                    for i in range(20):
                        route.append(random.choice(["S", "L", "R"]))
                    car_info[car_id]["route"] = route

                    car_info[car_id]["route_state"] = "NEW"
                    src_node_idx, dst_node_idx = src_dst_dict[car_id]
                    car_info[car_id]["src_node_idx"] = None
                    car_info[car_id]["intersection_manager"] = None
                    car_info[car_id]["dst_node_idx"] = dst_node_idx
                    car_info[car_id]["enter_time"] = simu_step
                    car_info[car_id]["car_length"] = traci.vehicle.getLength(car_id)


                is_handled = False
                for intersection_manager in intersection_manager_dict.values():
                    if (intersection_manager.check_in_my_region(lane_id) == "On my lane"):

                        current_turn = car_info[car_id]["route"][0]
                        next_turn = car_info[car_id]["route"][1]

                        intersection_manager.update_car(car_id, lane_id, simu_step, current_turn, next_turn)
                        is_handled = True
                        car_info[car_id]["inter_status"] = "On my lane"
                        car_info[car_id]["intersection_manager"] = intersection_manager

                    elif (intersection_manager.check_in_my_region(lane_id) == "In my intersection"):

                        # Check if the car enter the intersection (by changing state from "On my lane" to "in intersection")
                        if (car_info[car_id]["inter_status"] == "On my lane"):
                            car_info[car_id]["route"].pop(0)

                            # Dummy: Generate routes (turnings)
                            if len(car_info[car_id]["route"]) == 0:
                                route = []
                                for i in range(20):
                                    route.append(random.choice(["S", "L", "R"]))
                                car_info[car_id]["route"] = route


                        current_turn = car_info[car_id]["route"][0]
                        next_turn = car_info[car_id]["route"][1]

                        intersection_manager.update_car(car_id, lane_id, simu_step, current_turn, next_turn)
                        is_handled = True
                        car_info[car_id]["inter_status"] = "In my intersection"
                        car_info[car_id]["intersection_manager"] = intersection_manager

                    else:   # The intersection doesn't have the car
                        intersection_manager.delete_car(car_id)

                if not is_handled:
                    # Leaving intersections

                    traci.vehicle.setSpeed(car_id, cfg.MAX_SPEED)
                    car_info[car_id]["inter_status"] = "None"
                    car_info[car_id]["intersection_manager"] = None


            # Remove cars
            car_to_delete = []
            for car_id in car_info:
                if not car_id in all_c:
                    car_to_delete.append(car_id)
            for car_id in car_to_delete:
                del car_info[car_id]


            for intersection_manager in intersection_manager_dict.values():
                intersection_manager.run(simu_step)


            simu_step += cfg.TIME_STEP



    except Exception as e:
        traceback.print_exc()
        to_handler_queue.put("End Connection")


    #debug_t = threading.Thread(target=debug_ring)
    #debug_t.start()
    print(sys.argv[1], int(sys.argv[2]), int(sys.argv[3]), sys.argv[4])

    # Print out the measurements
    #print("Average total delay: ", total_delays/car_num)
    #print("Average delay by scheduling: ", total_delays_by_sche/car_num)
    print(intersection_manager.total_delays/intersection_manager.car_num, intersection_manager.total_delays_by_sche/intersection_manager.car_num, intersection_manager.car_num)

    print("avg_fuel = ",intersection_manager.total_fuel_consumption/intersection_manager.fuel_consumption_count)


    #'''
    file_name2 = 'result/result.csv'

    with open(file_name2, 'a', newline='') as csvfile2:
        writer2 = csv.writer(csvfile2, dialect='excel-tab', quoting=csv.QUOTE_MINIMAL, delimiter = ',')
        to_write2 = [sys.argv[1], sys.argv[2], sys.argv[3],
                    sys.argv[4], "_", simu_step
                    ]
        writer2.writerow(to_write2)
    #'''

    sys.stdout.flush()

    traci.close()



###########################
# Main function
if __name__ == "__main__":
    print("Usage: python code.py <arrival_rate (0~1.0)> <seed> <schedular> <grid_size n (nxn)>")
    sys.argv[4]

    print("Current routing period: ", cfg.ROUTING_PERIOD)

    # Initial variables
    seed = int(sys.argv[2])
    random.seed(seed)  # make tests reproducible
    numpy.random.seed(seed)
    arrival_rate = float(sys.argv[1])
    cfg.INTER_SIZE = int(sys.argv[4])

    # Initial SUMO
    sumoBinary = checkBinary('sumo-gui')

    # 0. Generate the intersection information files
    os.system("bash gen_intersection/gen_data.sh " + str(cfg.LANE_NUM_PER_DIRECTION))
    os.system("rm data/routes/*")

    # 1. Generate the route file for this simulation
    vehNr = generate_routefile_with_src_dst(cfg.INTER_SIZE, arrival_rate, seed, cfg.N_TIME_STEP)

    # Load from the file
    src_dst_file_name = "%i_%s_%i_src_dst.json" % (cfg.INTER_SIZE, arrival_rate, seed)
    with open('data/routes/'+src_dst_file_name) as json_file:
        src_dst_dict = json.load(json_file)


    handler_process = None
    try:
        # 3. Start TraCi
        net_name = "lane%iby%i.net.xml" % (cfg.INTER_SIZE, cfg.INTER_SIZE)
        route_name = "%i_%s_%i.rou.xml" % (cfg.INTER_SIZE, arrival_rate, seed)
        traci.start([sumoBinary, "-c", "data/UDTA.sumocfg",
                                 "--step-length", str(cfg.TIME_STEP),
                                 "--collision.mingap-factor", "0",
                                 "-n", "data/net/" + net_name,
                                 "-r", "data/routes/" + route_name])

        # 4. Echo and tell the size of the network
        handler_process, to_handler_queue, from_handler_queue = initial_server_handler(HOST, PORT)

        # 5. Start running SUMO
        run_sumo(handler_process, to_handler_queue, from_handler_queue, src_dst_dict)
    except Exception as e:
        traceback.print_exc()

    if handler_process != None:
        handler_process.terminate()
