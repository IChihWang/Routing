 #coding=utf-8

import math
import itertools
import heapq
import numpy
import sys
from multiprocessing import Process, Pool

from elements import Intersection_point, Car

def decide_available_turnings(src_coord, src_intersection_direction, dst_coord, additional_search_range):
    # additional_search_range: additional intersection number to be searched
    # Value of the dict: id of next intersection, the direction of next intersection
    available_turnings_and_out_direction = dict()
    if src_intersection_direction == 0:
        if dst_coord[0] - src_coord[0] > -additional_search_range:
            available_turnings_and_out_direction['R'] = ((src_coord[0]+1,src_coord[1]), 3)
        if src_coord[0] - dst_coord[0] > -additional_search_range:
            available_turnings_and_out_direction['L'] = ((src_coord[0]-1,src_coord[1]), 1)
        if dst_coord[1] - src_coord[1] > -additional_search_range:
            available_turnings_and_out_direction['S'] = ((src_coord[0],src_coord[1]+1), 0)
    elif src_intersection_direction == 1:
        if dst_coord[1] - src_coord[1] > -additional_search_range:
            available_turnings_and_out_direction['R'] = ((src_coord[0],src_coord[1]+1), 0)
        if src_coord[1] - dst_coord[1] > -additional_search_range:
            available_turnings_and_out_direction['L'] = ((src_coord[0],src_coord[1]-1), 2)
        if src_coord[0] - dst_coord[0] > -additional_search_range:
            available_turnings_and_out_direction['S'] = ((src_coord[0]-1,src_coord[1]), 1)
    elif src_intersection_direction == 2:
        if src_coord[0] - dst_coord[0] > -additional_search_range:
            available_turnings_and_out_direction['R'] = ((src_coord[0]-1,src_coord[1]), 1)
        if dst_coord[0] - src_coord[0] > -additional_search_range:
            available_turnings_and_out_direction['L'] = ((src_coord[0]+1,src_coord[1]), 3)
        if src_coord[1] - dst_coord[1] > -additional_search_range:
            available_turnings_and_out_direction['S'] = ((src_coord[0],src_coord[1]-1), 2)
    elif src_intersection_direction == 3:
        if src_coord[1] - dst_coord[1] > -additional_search_range:
            available_turnings_and_out_direction['R'] = ((src_coord[0],src_coord[1]-1), 2)
        if dst_coord[1] - src_coord[1] > -additional_search_range:
            available_turnings_and_out_direction['L'] = ((src_coord[0],src_coord[1]+1), 0)
        if dst_coord[0] - src_coord[0] > -additional_search_range:
            available_turnings_and_out_direction['S'] = ((src_coord[0]+1,src_coord[1]), 3)

    return available_turnings_and_out_direction # Key: turnings, Values: out_direction


def routing(grid_size, cars, database, GZ_BZ_CCZ_len, scheduling_period, V_MAX, TOTAL_LEN):

    route_record = dict()
    for car in cars:
        # Variables
        nodes_arrival_time_data = dict()         # ( node_coord, (Arrival time, last_node, recordings) )
        dst_coord = car.dst_coord           # The destination of the car

        # Initialization
        src_node = (car.src_coord, car.direction_of_src_intersection)
        nodes_arrival_time_data[src_node] = ( (0 + car.time_offset_step), None, None, [] )  # Starting with an offset, last_node, recordings
        is_visited_nodes = []

        unvisited_queue = [(nodes_arrival_time_data[src_node][0], src_node, car.position_at_offset)]

        # Routing
        while len(unvisited_queue) > 0:
            current_arrival_time, current_node, position_at_offset = heapq.heappop(unvisited_queue)

            # Skip if the node is visited, prevent multiple push into the heap
            if current_node in is_visited_nodes:
                continue

            is_visited_nodes.append(current_node)

            intersection_id = current_node[0]       # Current intersection id
            intersection_dir = current_node[1]      # Current intersection directino

            # Terminate when finding shortest path
            if intersection_id == car.dst_coord:
                car.traveling_time = current_arrival_time*scheduling_period + position_at_offset/V_MAX + TOTAL_LEN/V_MAX # additional time for car to leave sumo
                car.dst_node = current_node
                break

            # Get information from database
            intersection = database[current_arrival_time][intersection_id]

            # Decide the turnings
            available_turnings_and_out_direction = decide_available_turnings(intersection_id, intersection_dir, dst_coord, 0)

            for turning, out_data in available_turnings_and_out_direction.items():
                # Recording the states for final path
                recordings = []

                out_intersection_id, out_intersection_direction = out_data
                car.current_turn = turning
                car.lane = intersection.manager.advise_lane(car)
                car.position = position_at_offset

                # Determine the time arrive in Grouping Zone
                time_in_GZ = current_arrival_time
                while position_at_offset > GZ_BZ_CCZ_len:
                    # Record the path for final path retrieval
                    ###############################
                    car.position = position_at_offset
                    record_car_advising = car.copy_car_for_database()
                    recordings.append( (time_in_GZ, "lane_advising", record_car_advising) )
                    ###############################

                    time_in_GZ += 1
                    position_at_offset -= scheduling_period*V_MAX

                intersection_GZ = database[time_in_GZ][intersection_id]
                result = intersection_GZ.is_GZ_full(car, position_at_offset)
                while result[0] == False:
                    # The intersection is full

                    # Record the path for final path retrieval
                    ###############################
                    car.position = position_at_offset
                    record_car_advising = car.copy_car_for_database()
                    recordings.append( (time_in_GZ, "lane_advising", record_car_advising) )
                    ###############################

                    time_in_GZ += 1
                    intersection_GZ = database[time_in_GZ][intersection_id]
                    result = intersection_GZ.is_GZ_full(car, position_at_offset)

                position_at_offset = result[1]
                car.position = position_at_offset
                car_exiting_time = intersection_GZ.manager.run(car)

                # Record the path for final path retrieval
                ###############################
                record_car_scheduling = car.copy_car_for_database()
                recordings.append( (time_in_GZ, "scheduling", record_car_scheduling) )
                ###############################

                next_time_step = time_in_GZ + (int(car_exiting_time // scheduling_period) + 1)
                next_position_at_offset = TOTAL_LEN - ((int(car_exiting_time // scheduling_period) + 1)*scheduling_period - car_exiting_time) * V_MAX

                next_node = (out_intersection_id, out_intersection_direction)

                if next_node not in nodes_arrival_time_data or nodes_arrival_time_data[next_node][0] > next_time_step:
                    nodes_arrival_time_data[next_node] = (next_time_step, current_node, turning, recordings)
                    heapq.heappush(unvisited_queue, (next_time_step, next_node, next_position_at_offset))

        # Retrieve the paths
        path_list = []
        node = car.dst_node
        time, pre_node, turning, recordings = nodes_arrival_time_data[node]
        while pre_node != None:
            path_list.insert(0, (turning, recordings))

        route_record[car.id] = path_list



        # TODO: update the car route into the database



    return route_record

def add_car_to_database():
    # TODO: add car to database
    # TODO: extend database if not enough
    pass


class MiniVnet:
    def __init__(self, N, scheduling_period, routing_period_num, GZ_BZ_CCZ_len, HEADWAY, V_MAX, TOTAL_LEN):
        self.is_compiled = False
        self.init_time_length = 10
        self.N = N
        self.GZ_BZ_CCZ_len = GZ_BZ_CCZ_len

        self.scheduling_period = scheduling_period          # Perid for Scheduling of the intersection
        self.step_size = routing_period_num                 # Step size that the datacenter need to take
        self.routing_period = scheduling_period*routing_period_num  # Period for routing
        self.HEADWAY = HEADWAY
        self.V_MAX = V_MAX
        self.TOTAL_LEN = TOTAL_LEN

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
                intersection = Intersection_point((idx, jdx), self.GZ_BZ_CCZ_len, self.HEADWAY)
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

        input_data = [(self.N, route_groups[idx], self.database, self.GZ_BZ_CCZ_len, self.scheduling_period, self.V_MAX, self.TOTAL_LEN) for idx in range(process_num)]
        results = pool.starmap(routing, input_data)

        print(self.car_dict)
        # TODO: merge results
