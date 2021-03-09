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

        route_request = 'car_0,Pause;car_1,Pause;car_2,Pause;car_4,Pause;car_3,Pause;car_5,Pause;car_6,Pause;car_7,Pause;car_8,Pause;car_9,Pause;car_10,Pause;car_11,NEW,5.0,002_001,2,5,182.61,4;car_12,NEW,6.0,001_001,1,5,192.79,2;car_13,NEW,6.0,001_001,1,5,176.01,3;car_14,NEW,9.0,001_002,1,5,189.79,0;car_15,NEW,6.0,002_002,0,5,192.93,1;car_16,NEW,5.0,001_002,0,6,179.97,1;car_18,NEW,5.0,002_001,2,6,179.97,4;car_17,NEW,5.0,001_002,0,5,189.45,6;car_19,NEW,9.0,001_002,3,6,175.99,0;car_20,NEW,9.0,002_001,1,6,181.55,1;car_21,NEW,8.0,002_001,2,6,193.73,6;car_22,NEW,5.0,001_002,1,7,199.77,1;car_23,NEW,7.0,001_001,1,7,180.91,3;car_24,NEW,7.0,001_001,3,8,193.49,6;car_25,NEW,9.0,002_002,1,8,182.09,5;car_26,NEW,8.0,002_001,2,8,182.87,0;car_27,NEW,8.0,001_002,0,8,177.41,4;car_28,NEW,5.0,002_001,3,9,189.05,4;car_29,NEW,8.0,001_002,1,9,184.71,2;car_30,NEW,5.0,001_002,0,9,194.41,1;car_31,NEW,8.0,002_001,3,10,177.73,1;car_32,NEW,8.0,001_001,1,10,177.73,4;car_34,NEW,6.0,002_001,0,10,190.91,0;car_35,NEW,6.0,001_001,3,10,190.91,4;car_33,NEW,9.0,001_002,1,10,193.50,5;car_36,NEW,9.0,001_001,0,10,176.61,7;car_37,NEW,6.0,001_001,2,11,193.97,5;car_38,NEW,9.0,001_001,1,10,182.17,0;car_39,NEW,9.0,002_002,1,11,185.41,5;car_40,NEW,6.0,001_001,1,11,176.97,1;car_41,NEW,9.0,002_001,1,10,193.35,7;car_43,NEW,7.0,001_002,0,11,187.15,2;car_42,NEW,5.0,001_001,2,12,176.43,6;car_44,NEW,5.0,002_001,2,11,194.71,3;car_45,NEW,5.0,001_001,3,12,197.81,6;car_46,NEW,6.0,002_001,3,13,177.51,2;car_47,NEW,5.0,002_002,0,12,197.67,7;car_48,NEW,8.0,002_001,3,13,180.99,4;car_49,NEW,6.0,001_002,3,12,196.69,1;car_50,NEW,9.0,002_002,1,13,191.31,2;car_51,NEW,7.0,003_001,3,5,181.81,0;car_52,NEW,6.0,003_002,3,5,193.99,0;car_53,NEW,5.0,000_001,1,6,192.35,1;car_54,NEW,6.0,003_002,3,7,177.53,5;car_55,NEW,7.0,002_000,2,7,176.53,0;car_56,NEW,6.0,000_001,1,7,188.71,4;car_57,NEW,9.0,002_003,0,7,185.71,1;car_58,NEW,7.0,002_000,2,7,191.07,0;car_59,NEW,8.0,003_001,3,7,197.89,0;car_60,NEW,9.0,001_000,2,7,196.89,5;car_61,NEW,9.0,000_002,1,7,196.89,5;car_62,NEW,5.0,000_001,1,8,187.07,7;'

        # Parse the requests
        route_request = route_request[:-1]  # Remove the ';' at the end
        car_request_string_list = sroute_request.split(';')

        for car_request_string in car_request_string_list:
            car_request_info_list = car_request_string.split(',')


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
