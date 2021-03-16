
import sys
import config as cfg
import traci
import threading
import copy


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
        self.my_road_info = [{'avail_len':cfg.TOTAL_LEN, 'delay':0} for i in range(4*cfg.LANE_NUM_PER_DIRECTION)] # For updating the available road info
        self.others_road_info = [None]*(4*cfg.LANE_NUM_PER_DIRECTION)   # Reference to read others road info
        self.spillback_delay_record = [0]*(4*cfg.LANE_NUM_PER_DIRECTION)


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


    def get_car_info_for_route(self, car_id):
        time_offset = None
        src_intersection_id = None              # After offset, which intersection/node
        direction_of_src_intersection = None
        position = self.car_list[car_id].position

        # Not yet enter Roadrunner
        if self.car_list[car_id].zone == None:
            diff_pos = position - cfg.TOTAL_LEN
            time_offset = diff_pos/cfg.MAX_SPEED

            if time_offset > cfg.ROUTING_PERIOD:
                # Get the origin intersection id
                lane = self.car_list[car_id].lane
                direction = lane//cfg.LANE_NUM_PER_DIRECTION

                direction_of_src_intersection = direction
                src_intersection_id = self.ID

            else:
                time_offset = None

        # 1. Not yet but about to enter the intersection region
        # 2. Already inside the intersection region
        if time_offset == None:
            # Find lane of the car
            lane = None
            if self.car_list[car_id].zone == "AZ":
                lane = self.car_list[car_id].desired_lane
            else:
                lane = self.car_list[car_id].lane

            # Compute the time_offset
            if not isinstance(self.car_list[car_id].D, float):
                # Delay is not computed yet
                time_offset = position/cfg.MAX_SPEED
                # Estimate by borrowing the known delay
                if len(self.my_road_info[lane]['car_delay_position']) > 0:
                    time_offset += self.my_road_info[lane]['car_delay_position'][-1]['delay']
            else:
                # Compute with the known delay
                time_offset = self.car_list[car_id].OT + self.car_list[car_id].D

            # Add the time in the intersection
            turning = self.car_list[car_id].current_turn
            time_in_inter = inter_length_data.getIntertime(lane, turning)
            time_offset += time_in_inter  # time passing intersection

            # Start from next intersection
            if time_offset <= cfg.ROUTING_PERIOD:
                # About to exit the intersection
                # Not allowing scheduling for a while
                return None
            else:
                direction = lane//cfg.LANE_NUM_PER_DIRECTION

                if turning == 'S':
                    direction_of_src_intersection = direction
                elif turning == 'L':
                    direction_of_src_intersection = (direction+1)%4
                elif turning == 'R':
                    direction_of_src_intersection = (direction-1)%4

                intersection_idx_list = self.ID.split("_")
                if direction_of_src_intersection == 0:
                    intersection_idx_list[1] = "%3.3o"%(int(intersection_idx_list[1])+1)
                elif direction_of_src_intersection == 1:
                    intersection_idx_list[0] = "%3.3o"%(int(intersection_idx_list[0])-1)
                elif direction_of_src_intersection == 2:
                    intersection_idx_list[1] = "%3.3o"%(int(intersection_idx_list[1])-1)
                elif direction_of_src_intersection == 3:
                    intersection_idx_list[0] = "%3.3o"%(int(intersection_idx_list[0])+1)

                src_intersection_id = intersection_idx_list[0] + "_" + intersection_idx_list[1]

        time_offset_step = int(time_offset//cfg.SCHEDULING_PERIOD+1)
        time_offset = time_offset_step*cfg.SCHEDULING_PERIOD - time_offset
        position_at_offset = cfg.TOTAL_LEN - time_offset*cfg.MAX_SPEED

        # Might return None as well for temporary forbid of routing
        return (position_at_offset, time_offset_step, src_intersection_id, direction_of_src_intersection)

    def delete_car(self, car_id):
        # TODO: rewrite
        if car_id in self.car_list:
            self.car_list.pop(car_id)


    def advise_lane(self, target_car):
        self.lane_advisor.updateTableFromCars(sched_cars, scheduling_cars+advising_car)

        # Check whether there is a spillback
        accumulate_car_len_lane = [0]*(4*cfg.LANE_NUM_PER_DIRECTION)
        spillback_lane_advise_avoid = [False]*(4*cfg.LANE_NUM_PER_DIRECTION)

        for car in self.sched_cars+self.scheduling_cars+self.advising_car:
            lane_idx = car.lane

            if self.others_road_info[lane_idx] != None:
                accumulate_car_len_lane[lane_idx] += (car.length + cfg.HEADWAY)

        for lane_idx in range(4*cfg.LANE_NUM_PER_DIRECTION):
            if self.others_road_info[lane_idx] != None:
                if accumulate_car_len_lane[lane_idx] >= self.others_road_info[lane_idx]['avail_len']:
                    spillback_lane_advise_avoid[lane_idx] = True

        advised_lane = self.lane_advisor.adviseLane(target_car, spillback_lane_advise_avoid)
        return advised_lane


    # Three group of cars in three zones
    def run(self, target_car):
        # Compute the OT for the car
        target_car.OT = target_car.position / cfg.MAX_SPEED

        self.scheduling_cars
        for car in self.scheduling_cars:
            car.D = None

        IcaccPlus(self.sched_car, self.scheduling_cars+[target_car], self.others_road_info, self.spillback_delay_record)

        car_exiting_time = target_car.OT + target_car.D

        turning = target_car.current_turn
        car_exiting_time += inter_length_data.getIntertime(target_car.lane, turning)

        return car_exiting_time

        '''
        for lane_idx in range(4*cfg.LANE_NUM_PER_DIRECTION):
            self.my_road_info[lane_idx]['avail_len'] = cfg.TOTAL_LEN - car_accumulate_len_lane[lane_idx] - cfg.HEADWAY
            self.my_road_info[lane_idx]['delay'] = delay_lane[lane_idx]
            self.my_road_info[lane_idx]['simu_step'] = simu_step
            self.my_road_info[lane_idx]['car_delay_position'] = lane_car_delay_position[lane_idx]
        '''
