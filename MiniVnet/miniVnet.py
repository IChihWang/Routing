 #coding=utf-8

import math
import itertools
import heapq
import numpy
import sys
from multiprocessing import Process, Pool

from elements import Intersection_point, Car


def routing(grid_size, cars, database):
    # TODO: routing among the cars

    for car in cars:
        # Variables
        nodes_arrival_time = dict()         # (node_coord, Arrival time)

        # Initialization
        src_node = (car.src_coord, car.direction_of_src_intersection)
        nodes_arrival_time[src_node] = (0 + car.time_offset_step)  # Starting with an offset

        unvisited_queue = [(nodes_arrival_time[src_node], src_node)]

        while len(unvisited_queue) > 0:
            current_arrival_time, current_node = heapq.heappop(unvisited_queue)

            intersection_id = current_node[0]
            intersection_dir = current_node[1]
            # Get information from database
            intersection = database[current_arrival_time][intersection_id]
            # Add the car into the intersection info

            # Do the sequensing and then add queue


    pass

class MiniVnet:
    def __init__(self, N, scheduling_period, routing_period_num):
        self.is_compiled = False
        self.init_time_length = 10
        self.N = N

        self.scheduling_period = scheduling_period          # Perid for Scheduling of the intersection
        self.step_size = routing_period_num                 # Step size that the datacenter need to take

        self.database = self.create_grid_network(N, 3)

        self.car_dict = dict()


    #=============  Construct network functions ======================
    # Connect the intersection to prevent spillback
    def connect_intersections(self, N, intersection_map):
        for idx in range(N):
            for jdx in range(N):
                if idx <= N-2:
                    intersection_map[(idx, jdx)].connect(1, intersection_manager_dict[(idx+1, jdx)], 3)

                if jdx <= N-2:
                    intersection_map[(idx, jdx)].connect(2, intersection_manager_dict[(idx, jdx+1)], 0)

    # Add a time step into a database
    def add_time_step(self, N, database):
        intersection_map = dict()
        for idx in range(N):
            for jdx in range(N):
                intersection = Intersection_point((idx, jdx))
                intersection_map[(idx, jdx)] = intersection
        #self.connect_intersections(N, intersection_map)

        database.append(intersection_map)

    # Delete a time step from the database
    def delete_time_step(self, database):
        del database[-1]

    # Create a database for a grid network
    def create_grid_network(self, N, num_lane):
        database = []

        for time_idx in range(self.init_time_length):
            self.add_time_step(N, database)

        return database


    #=============  Update car information from SUMO ======================
    def update_car(self, car_id, car_length, src_intersection_id,
                        direction_of_src_intersection, time_offset_step,
                        position_at_offset, dst_node_id):
        # Create Car object if it is not in dict
        if car_id not in self.car_dict:
            dst_coord_list = dst_node_id.split('_')
            dst_coord = (int(dst_coord_list[0]), int(dst_coord_list[1]))
            self.car_dict[car_id] = Car(car_id, car_length, dst_coord)

        # Update the source
        src_coord_list = src_intersection_id.split('_')
        src_coord = (int(src_coord_list[0]), int(src_coord_list[1]))
        self.car_dict[car_id].src_coord = src_coord
        self.car_dict[car_id].direction_of_src_intersection = direction_of_src_intersection
        self.car_dict[car_id].time_offset_step = time_offset_step
        self.car_dict[car_id].position_at_offset = position_at_offset


    def delete_car_from_database(self, car_id):
        # TODO: delete car from the database
        pass


    def choose_car_to_thread_group(self, process_num, new_cars_id, old_cars_id):
        # TODO: temp, route all new cars with two threads
        # TODO: Clear cars for chosen cars

        result = []
        for car_id in new_cars_id:
            result.append(self.car_dict[car_id])
        return [result, [], [], [], []]



    def routing_with_groups(self, process_num, route_groups):
        pool = Pool(process_num)

        input_data = [(self.N, route_groups[idx], self.database) for idx in range(process_num)]
        results = pool.starmap(routing, input_data)

        print(self.car_dict)
        # TODO: merge results
