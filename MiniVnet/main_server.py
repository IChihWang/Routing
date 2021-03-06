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

            from_handler_queue.put(data)


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

        while is_continue:
            route_request = from_handler_queue.get()

            if route_request == "End Connection":
                break

            to_handler_queue.put(route_request)

    except Exception as e:
        traceback.print_exc()


if __name__ == '__main__':
    print("Usage: python code.py <choose_car_algorithm> <iteration_num> <thread_num> <Top N number>")
    print("--------------------- <arrival_rate> <rand_seed> <grid_size>")
    sys.argv[4]

    handler_process = None
    try:
        # 1. Echo and tell the size of the network
        handler_process, to_handler_queue, from_handler_queue, grid_size, scheduling_period, routing_period_num = initial_server_handler(HOST, PORT)

        # 2. Initialize the router
        #router = MiniVnet(grid_size, scheduling_period, routing_period_num)

        # 3. Start running SUMO
        run_router(None, handler_process, to_handler_queue, from_handler_queue)

    except Exception as e:
        traceback.print_exc()

    handler_process.terminate()
