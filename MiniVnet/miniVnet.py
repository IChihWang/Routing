 #coding=utf-8

import math
import itertools
import heapq
import numpy
import sys
import time as time_lib
from multiprocessing import Process, Pool

from elements import Intersection_point, Car

def decide_available_turnings(src_coord, src_intersection_direction, dst_coord, grid_size, additional_search_range):
    # additional_search_range: additional intersection number to be searched
    # Value of the dict: id of next intersection, the direction of next intersection
    available_turnings_and_out_direction = dict()
    if src_intersection_direction == 0:
        if dst_coord[0] - src_coord[0] > -additional_search_range:
            if src_coord[0] < grid_size or src_coord[1] == dst_coord[1]:
                available_turnings_and_out_direction['R'] = ((src_coord[0]+1,src_coord[1]), 3)
        if src_coord[0] - dst_coord[0] > -additional_search_range:
            if src_coord[0] > 1 or src_coord[1] == dst_coord[1]:
                available_turnings_and_out_direction['L'] = ((src_coord[0]-1,src_coord[1]), 1)
        if dst_coord[1] - src_coord[1] > -additional_search_range:
            if src_coord[1] < grid_size or src_coord[0] == dst_coord[0]:
                available_turnings_and_out_direction['S'] = ((src_coord[0],src_coord[1]+1), 0)
    elif src_intersection_direction == 1:
        if dst_coord[1] - src_coord[1] > -additional_search_range:
            if src_coord[1] < grid_size or src_coord[0] == dst_coord[0]:
                available_turnings_and_out_direction['R'] = ((src_coord[0],src_coord[1]+1), 0)
        if src_coord[1] - dst_coord[1] > -additional_search_range:
            if src_coord[1] > 1 or src_coord[0] == dst_coord[0]:
                available_turnings_and_out_direction['L'] = ((src_coord[0],src_coord[1]-1), 2)
        if src_coord[0] - dst_coord[0] > -additional_search_range:
            if src_coord[0] > 1 or src_coord[1] == dst_coord[1]:
                available_turnings_and_out_direction['S'] = ((src_coord[0]-1,src_coord[1]), 1)
    elif src_intersection_direction == 2:
        if src_coord[0] - dst_coord[0] > -additional_search_range:
            if src_coord[0] > 1 or src_coord[1] == dst_coord[1]:
                available_turnings_and_out_direction['R'] = ((src_coord[0]-1,src_coord[1]), 1)
        if dst_coord[0] - src_coord[0] > -additional_search_range:
            if src_coord[0] < grid_size or src_coord[1] == dst_coord[1]:
                available_turnings_and_out_direction['L'] = ((src_coord[0]+1,src_coord[1]), 3)
        if src_coord[1] - dst_coord[1] > -additional_search_range:
            if src_coord[1] > 1 or src_coord[0] == dst_coord[0]:
                available_turnings_and_out_direction['S'] = ((src_coord[0],src_coord[1]-1), 2)
    elif src_intersection_direction == 3:
        if src_coord[1] - dst_coord[1] > -additional_search_range:
            if src_coord[1] > 1 or src_coord[0] == dst_coord[0]:
                available_turnings_and_out_direction['R'] = ((src_coord[0],src_coord[1]-1), 2)
        if dst_coord[1] - src_coord[1] > -additional_search_range:
            if src_coord[1] < grid_size or src_coord[0] == dst_coord[0]:
                available_turnings_and_out_direction['L'] = ((src_coord[0],src_coord[1]+1), 0)
        if dst_coord[0] - src_coord[0] > -additional_search_range:
            if src_coord[0] < grid_size or src_coord[1] == dst_coord[1]:
                available_turnings_and_out_direction['S'] = ((src_coord[0]+1,src_coord[1]), 3)

    return available_turnings_and_out_direction # Key: turnings, Values: out_direction


def routing(miniVnet, cars):
    grid_size = miniVnet.N
    database = miniVnet.database
    GZ_BZ_CCZ_len = miniVnet.GZ_BZ_CCZ_len
    scheduling_period = miniVnet.scheduling_period
    routing_period = miniVnet.routing_period
    V_MAX = miniVnet.V_MAX
    TOTAL_LEN = miniVnet.TOTAL_LEN
    LANE_NUM_PER_DIRECTION = miniVnet.LANE_NUM_PER_DIRECTION
    TURN_SPEED = miniVnet.TURN_SPEED

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
            intersection = miniVnet.get_intersection(current_arrival_time, intersection_id)

            # Decide the turnings
            available_turnings_and_out_direction = decide_available_turnings(intersection_id, intersection_dir, dst_coord, grid_size, 0)

            for turning, out_data in available_turnings_and_out_direction.items():
                # Recording the states for final path
                recordings = []

                out_intersection_id, out_intersection_direction = out_data
                car.lane = intersection_dir * miniVnet.LANE_NUM_PER_DIRECTION
                car.current_turn = turning
                car.lane = intersection.manager.advise_lane(car)
                car.position = position_at_offset
                car.update_dst_lane_and_data(LANE_NUM_PER_DIRECTION, V_MAX, TURN_SPEED)

                # Determine the time arrive in Grouping Zone
                time_in_GZ = current_arrival_time
                while position_at_offset > GZ_BZ_CCZ_len:
                    # Record the path for final path retrieval
                    ###############################
                    car.position = position_at_offset
                    record_car_advising = car.copy_car_for_database()
                    recordings.append( (time_in_GZ, intersection_id, "lane_advising", record_car_advising) )
                    ###############################

                    time_in_GZ += 1
                    position_at_offset -= scheduling_period*V_MAX

                intersection_GZ = miniVnet.get_intersection(time_in_GZ, intersection_id)
                result = intersection_GZ.is_GZ_full(car, position_at_offset)
                position_at_offset = result[1]
                while result[0] == False:
                    # The intersection is full

                    # Record the path for final path retrieval
                    ###############################
                    car.position = position_at_offset
                    record_car_advising = car.copy_car_for_database()
                    recordings.append( (time_in_GZ, intersection_id, "lane_advising", record_car_advising) )
                    ###############################

                    time_in_GZ += 1
                    intersection_GZ = miniVnet.get_intersection(time_in_GZ, intersection_id)
                    result = intersection_GZ.is_GZ_full(car, position_at_offset)
                    position_at_offset = result[1]

                position_at_offset = result[1]
                car.position = position_at_offset
                car_exiting_time = intersection_GZ.manager.run(car)


                while car_exiting_time == None or result[0] == False:
                    # The scheduling is prosponed due to spillback

                    # Record the path for final path retrieval
                    ###############################
                    record_car_advising = car.copy_car_for_database()
                    recordings.append( (time_in_GZ, intersection_id, "lane_advising", record_car_advising) )
                    ###############################

                    time_in_GZ += 1
                    intersection_GZ = miniVnet.get_intersection(time_in_GZ, intersection_id)
                    result = intersection_GZ.is_GZ_full(car, position_at_offset)

                    if result[0] == True:
                        position_at_offset = result[1]
                        car.position = position_at_offset
                        car_exiting_time = intersection_GZ.manager.run(car)


                # Record the path for final path retrieval
                ###############################
                record_car_scheduling = car.copy_car_for_database()
                recordings.append( (time_in_GZ, intersection_id, "scheduling", record_car_scheduling) )
                ###############################

                next_time_step = time_in_GZ + int(car_exiting_time // scheduling_period)
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
            path_list.insert(0, (turning, recordings, time))
            time, pre_node, turning, recordings = nodes_arrival_time_data[pre_node]

        route_record[car.id] = path_list

        miniVnet.add_car_to_database(car, path_list)

    return route_record


class MiniVnet:
    def __init__(self, N, scheduling_period, routing_period_num, GZ_BZ_CCZ_len, HEADWAY, V_MAX, TURN_SPEED, TOTAL_LEN):
        self.is_compiled = False
        self.init_time_length = 10
        self.N = N
        self.GZ_BZ_CCZ_len = GZ_BZ_CCZ_len

        self.scheduling_period = scheduling_period          # Perid for Scheduling of the intersection
        self.step_size = routing_period_num                 # Step size that the datacenter need to take
        self.routing_period = scheduling_period*routing_period_num  # Period for routing
        self.HEADWAY = HEADWAY
        self.V_MAX = V_MAX
        self.TURN_SPEED = TURN_SPEED
        self.TOTAL_LEN = TOTAL_LEN
        self.LANE_NUM_PER_DIRECTION = 3

        self.database = self.create_grid_network(self.LANE_NUM_PER_DIRECTION)

        self.car_dict = dict()


    #=============  Construct network functions ======================
    # Connect the intersection to prevent spillback
    def connect_intersections(self, N, intersection_map):
        for idx in range(1, N+1):
            for jdx in range(1, N+1):
                if idx <= N-1:
                    intersection_map[(idx, jdx)].connect(1, intersection_map[(idx+1, jdx)], 3)

                if jdx <= N-1:
                    intersection_map[(idx, jdx)].connect(2, intersection_map[(idx, jdx+1)], 0)


    # Get an intersection from database
    def get_intersection(self, current_arrival_time, intersection_id):
        database = self.database
        while current_arrival_time >= len(database):
            self.add_time_step(database)

        intersection = database[current_arrival_time][intersection_id]
        return intersection


    def add_car_to_database(self, target_car, path_list):
        recordings = []
        for path_data in path_list:
            recordings += path_data[1]

        pre_record = None

        for record in recordings:
            time, intersection_id, state, car = record
            intersection = self.get_intersection(time, intersection_id)

            # See if the record change to next intersection: add scheduled cars
            if pre_record != None and intersection_id != pre_record[1]:
                pre_time = pre_record[0]
                pre_car = pre_record[2]
                saving_car = pre_car
                for time_idx in range(pre_time+1, time):
                    saving_car = saving_car.copy_car_for_database()
                    saving_car.OT -= self.scheduling_period


                    if saving_car.OT+saving_car.D > 0:
                        intersection_to_save = self.get_intersection(time_idx, intersection_id)
                        intersection_to_save.add_sched_car(saving_car)
                        target_car.records_intersection_in_database.append( ("scheduled", intersection_to_save) )

            pre_record = (time, intersection_id, car)

            if state == "lane_advising":
                saving_car = car.copy_car_for_database()
                intersection.add_advising_car(saving_car)
                target_car.records_intersection_in_database.append( ("lane_advising", intersection) )

            elif state == "scheduling":
                saving_car = car.copy_car_for_database()
                intersection.add_scheduling_cars(saving_car)
                target_car.records_intersection_in_database.append( ("scheduling", intersection) )



        # Add the scheduled car before exiting to the database
        exiting_time = path_list[-1][2]
        pre_time = pre_record[0]
        intersection_id = pre_record[1]
        pre_car = pre_record[2]
        saving_car = pre_car
        for time_idx in range(pre_time+1, exiting_time):
            saving_car = saving_car.copy_car_for_database()
            saving_car.OT -= self.scheduling_period

            if saving_car.OT+saving_car.D > 0:
                intersection_to_save = self.get_intersection(time_idx, intersection_id)
                intersection_to_save.add_sched_car(saving_car)
                target_car.records_intersection_in_database.append( ("scheduled", intersection_to_save) )


    # Move a time step
    def move_a_time_step(self):
        for idx in range(self.step_size):
            self.delete_time_step()

    # Delete a time step from the database
    def delete_time_step(self):
        if len(self.database) > 0:
            del self.database[0]

    # Add a time step into a database
    def add_time_step(self, database):
        intersection_map = dict()
        for idx in range(1, self.N + 1):
            for jdx in range(1, self.N + 1):
                intersection = Intersection_point((idx, jdx), self.GZ_BZ_CCZ_len, self.HEADWAY, self.TOTAL_LEN)
                intersection_map[(idx, jdx)] = intersection
        self.connect_intersections(self.N, intersection_map)

        database.append(intersection_map)

    # Create a database for a grid network
    def create_grid_network(self, num_lane):
        database = []

        for time_idx in range(self.init_time_length):
            self.add_time_step(database)

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
        src_coord = (int(src_coord_list[0]), int(src_coord_list[1]))    # Move intersection id to (0,0)
        self.car_dict[car_id].src_coord = src_coord
        self.car_dict[car_id].direction_of_src_intersection = direction_of_src_intersection
        self.car_dict[car_id].time_offset_step = time_offset_step
        self.car_dict[car_id].position_at_offset = position_at_offset


    def delete_car_from_database(self, car):
        for type, intersection in car.records_intersection_in_database:
            intersection.delete_car_from_database(car, type)

        car.records_intersection_in_database = []

    def delete_car_from_database_id(self, car_id):
        self.delete_car_from_database(self.car_dict[car_id])
        del self.car_dict[car_id]

    def choose_car_to_thread_group(self, process_num, new_cars_id, old_cars_id):
        # TODO: write better algorithm

        result = [[] for idx in range(process_num)]
        process_idx = 0
        for car_id in new_cars_id:
            process_idx += 1
            process_idx %= process_num
            result[process_idx].append(self.car_dict[car_id])

        for car_group in result:
            for car in car_group:
                self.delete_car_from_database(car)

        return result

    def update_database_after_routing(self, route_groups):
        for car_group in route_groups:
            for car in car_group:
                # Spillback info of each intersection is updated in this function
                self.add_car_to_database(car, car.path_data)

    def routing_with_groups(self, process_pool, process_num, route_groups, out_route_dict):

        input_data = [(self, route_groups[idx]) for idx in range(process_num)]
        results = process_pool.starmap(routing, input_data)

        for result in results:
            for car_id, path_data in result.items():
                turnings_str = ""
                for turning, recordings, time in path_data:
                    turnings_str += turning
                self.car_dict[car_id].path_data = path_data

                out_route_dict[car_id] = turnings_str

        return out_route_dict
