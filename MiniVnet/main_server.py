# Create a network as following:
#         s1  s2
#
#   s8    i1  i2    s3
#   s7    i3  i4    s4
#
#         s6  s5
#
# Index of the initersection:
#       1
#   0   I   2
#       3
#


import random
import numpy as np

import time
import threading
import socket
import sys
import csv

import traceback
import multiprocessing
from multiprocessing import Manager, Value
import logging
#from miniVnet import MiniVnet

random.seed(0)
np.random.seed(0)
sys.setrecursionlimit(50000)
update_lock = threading.Lock()

# Initial the client
HOST, PORT = "localhost", 9996

def initial_server_handler(HOST, PORT):
    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_sock.bind((HOST, PORT))
    print("start server")

    server_sock.listen(5)
    sock, addr = server_sock.accept()
    print("Got a connection from: ", addr)

    hello_msg = sock.recv(1024).decode().split(":")
    print(hello_msg)
    grid_size = int(hello_msg[1])
    scheduling_period = float(hello_msg[3])
    routing_period_num = int(hello_msg[5])    # The step it need to shift in database
    sock.send("Got it ;@".encode())

    to_handler_queue = multiprocessing.Queue()
    from_handler_queue = multiprocessing.Queue()

    handler_process = multiprocessing.Process(target=handler, args=(sock, to_handler_queue, from_handler_queue))
    handler_process.start()

    return (handler_process, to_handler_queue, from_handler_queue, grid_size, scheduling_period, routing_period_num)

def handler(sock, to_handler_queue, from_handler_queue):
    try:
        is_continue = True

        while(is_continue):
            # Receive the result
            data = ""
            while len(data) == 0 or data[-1] != "@":
                get_str = sock.recv(8192)
                if get_str == b'':
                    from_handler_queue.put("End Connection")
                    break
                data += get_str.decode()

            from_handler_queue.put(data[:-1])       # Remove the "@" at the end


            # Get data from Router part
            send_str = to_handler_queue.get()
            # ===========   Block   ================
            if send_str == "end":
                is_continue = False
                break

            # Send route
            send_str = "Echo: " + send_str + "@"
            sock.sendall(send_str.encode())

    except Exception as e:
        traceback.print_exc()
        from_handler_queue.put("End Connection")

    sock.close()



def run_router(router, _handler_process, _to_handler_queue, _from_handler_queue):
    simu_step = 0

    handler_process = _handler_process
    to_handler_queue = _to_handler_queue
    from_handler_queue = _from_handler_queue

    new_car_list = []
    old_car_list = []

    try:
        is_continue = True      # Whether it should continue listen to requests

        '''
        while is_continue:
            route_request = from_handler_queue.get()

            if route_request == "End Connection":
                break

            #TODO route !
            to_handler_queue.put(route_request)
        '''

        '''
        route_request = 'car_0,NEW,9.0,003_001,3,9,183.70,003_001;car_1,NEW,8.0,003_002,3,10,198.97,003_002;car_2,NEW,8.0,001_003,0,11,185.15,001_000;car_3,NEW,9.0,001_001,1,11,184.02,003_002;car_4,NEW,7.0,002_003,0,10,194.24,000_001;car_5,NEW,9.0,002_002,0,11,184.02,001_003;car_6,NEW,6.0,002_002,0,12,184.38,001_000;car_7,NEW,9.0,002_000,2,14,176.23,000_001;car_8,NEW,5.0,003_002,3,13,199.61,002_000;car_10,NEW,9.0,001_001,0,5,178.70,002_000;car_11,NEW,6.0,001_002,3,5,181.70,002_000;car_9,NEW,6.0,002_001,1,5,181.70,003_002;car_12,NEW,5.0,001_001,3,5,193.88,003_001;car_14,NEW,5.0,002_002,1,5,193.88,002_003;car_13,NEW,5.0,001_002,3,5,194.99,002_000;car_15,NEW,5.0,002_001,1,6,180.06,000_002;car_16,NEW,6.0,001_001,0,6,190.24,001_003;car_17,NEW,8.0,002_002,2,6,188.24,003_001;car_18,NEW,8.0,002_002,1,6,188.24,002_003;car_19,NEW,5.0,002_001,1,7,177.42,002_000;car_20,NEW,7.0,001_002,3,7,175.42,003_002;car_21,NEW,9.0,001_001,3,7,195.78,001_003;car_22,NEW,6.0,001_002,2,7,198.78,000_001;car_23,NEW,8.0,002_001,1,8,182.96,002_003'

        # Parse the requests
        route_request = route_request[:-1]  # Remove the ';' at the end
        car_request_string_list = sroute_request.split(';')

        for car_request_string in car_request_string_list:
            car_request_info_list = car_request_string.split(',')
            car_id = car_request_info_list[0]
            if car_request_info_list[1] == "EXIT":
                # TODO: Remove the car from the database
                router.delete_car_from_database(car_id)
                pass
            elif car_request_info_list[1] == "PAUSE":
                # Cannot reroute the car due to the lower lever control
                # TODO: considering update info
                pass
            elif car_request_info_list[1] == "NEW" or car_request_info_list[1] == "OLD":
                # Must do the routing for the car
                car_length = int(car_request_info_list[2])
                src_intersection_id = car_request_info_list[3]
                direction_of_src_intersection = int(car_request_info_list[4])
                time_offset_step = int(car_request_info_list[5])
                position_at_offset = float(car_request_info_list[6])
                dst_node_idx = int(car_request_info_list[7])

                router.update_car(car_id, car_length, src_intersection_id,
                                    direction_of_src_intersection, time_offset_step,
                                    position_at_offset, dst_node_idx)

                if car_request_info_list[1] == "NEW":
                    new_car_list.append(car_id)
                elif car_request_info_list[1] == "OLD":
                    old_car_list.append(car_id)

        print(route_request)


    except Exception as e:
        traceback.print_exc()


if __name__ == '__main__':
    print("Usage: python code.py <choose_car_algorithm> <iteration_num> <thread_num> <Top N number>")
    print("--------------------- <arrival_rate> <rand_seed> <grid_size>")
    sys.argv[4]

    handler_process = None
    try:
        # 1. Echo and tell the size of the network
        #handler_process, to_handler_queue, from_handler_queue, grid_size, scheduling_period, routing_period_num = initial_server_handler(HOST, PORT)

        # 2. Initialize the router
        #router = MiniVnet(grid_size, scheduling_period, routing_period_num)

        # 3. Start running SUMO
        #run_router(None, handler_process, to_handler_queue, from_handler_queue)
        run_router(None, None, None, None)

    except Exception as e:
        traceback.print_exc()

    if handler_process != None:
        handler_process.terminate()
