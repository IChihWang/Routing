
import sys
import os
import config as cfg
import threading
import copy


sys.path.append('/usr/share/sumo/tools/')
# we need to import python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")


import traci


from Cars import Car
from milp import Icacc, IcaccPlus, Fcfs, FixedSignal
from LaneAdviser import LaneAdviser
from get_inter_length_info import Data

inter_length_data = Data()

class IntersectionManager:
    def __init__(self, id):
        self.ID = id
        self.lane_advisor = LaneAdviser()

        self.sched_cars = dict()
        self.scheduling_cars = dict()
        self.advising_car = dict()

        # Spill-back info
        self.my_road_info = [{'avail_len':cfg.TOTAL_LEN, 'delay':0, 'car_delay_position':[]} for i in range(4*cfg.LANE_NUM_PER_DIRECTION)] # For updating the available road info
        self.others_road_info = [None]*(4*cfg.LANE_NUM_PER_DIRECTION)   # Reference to read others road info


    def connect(self, my_direction, intersection, its_direction):
        '''
                2
            3   i   1
                0
        ''' #(Lane index are all counter-clockwise)
        for lane_idx in range(cfg.LANE_NUM_PER_DIRECTION):
            my_lane = my_direction*cfg.LANE_NUM_PER_DIRECTION+lane_idx
            its_lane = its_direction*cfg.LANE_NUM_PER_DIRECTION+(cfg.LANE_NUM_PER_DIRECTION-lane_idx-1)

            self.others_road_info[my_lane] = intersection.my_road_info[its_lane]
            intersection.others_road_info[its_lane] = self.my_road_info[my_lane]



    def delete_car(self, car_id):
        if car_id in self.car_list:
            self.car_list.pop(car_id)


    def advise_lane(self, target_car):
        sched_cars = list(self.sched_cars.values())
        scheduling_cars = list(self.scheduling_cars.values())
        advising_car = list(self.advising_car.values())

        self.lane_advisor.updateTableFromCars(sched_cars, scheduling_cars+advising_car)

        # Check whether there is a spillback
        accumulate_car_len_lane = [0]*(4*cfg.LANE_NUM_PER_DIRECTION)
        spillback_lane_advise_avoid = [False]*(4*cfg.LANE_NUM_PER_DIRECTION)

        for car in sched_cars+scheduling_cars+advising_car:
            lane_idx = car.lane

            if self.others_road_info[lane_idx] != None:
                accumulate_car_len_lane[lane_idx] += (car.length + cfg.HEADWAY)

        for lane_idx in range(4*cfg.LANE_NUM_PER_DIRECTION):
            if self.others_road_info[lane_idx] != None:
                if accumulate_car_len_lane[lane_idx] >= self.others_road_info[lane_idx]['avail_len']:
                    spillback_lane_advise_avoid[lane_idx] = True

        advised_lane = self.lane_advisor.adviseLane(target_car, spillback_lane_advise_avoid)
        return advised_lane


    # Do scheduling (return None if the scheduling is prosponed)
    def run(self, target_car):
        sched_cars = list(self.sched_cars.values())
        scheduling_cars = list(self.scheduling_cars.values())

        # Compute the OT for the car
        target_car.OT = target_car.position / cfg.MAX_SPEED
        target_car.D = None

        for car in scheduling_cars:
            car.D = None

        IcaccPlus(sched_cars, scheduling_cars+[target_car], self.others_road_info, target_car)

        if target_car.is_spillback_strict == True or target_car.D == None:
            # Prospond the scheduling due to spillback, return None
            # (1) When the car is blocked by the spillback (2) Its front car is blocked by the spillback
            return None

        else:
            car_exiting_time = target_car.OT + target_car.D

            turning = target_car.current_turn
            car_exiting_time += inter_length_data.getIntertime(target_car.lane, turning)

            return car_exiting_time
