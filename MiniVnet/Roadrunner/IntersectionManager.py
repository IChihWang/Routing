
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
        self.az_list = dict()
        self.pz_list = dict()

        self.car_list = dict()   # Cars that needs to be handled
        self.cc_list = dict()    # Cars under Cruse Control in CCZ
        self.leaving_cars = dict()   # Cars just entered the intersection (leave the CC zone)

        self.schedule_period_count = 0
        self.lane_advisor = LaneAdviser()
        self.scheduling_thread = None
        self.in_lanes = []
        self.out_lanes = []

        # For front car
        self.CC_last_cars_on_lanes = dict()
        for idx in range(4*cfg.LANE_NUM_PER_DIRECTION):
            self.CC_last_cars_on_lanes[idx] = None


        # Statistics
        self.total_delays = 0
        self.total_delays_by_sche = 0
        self.car_num = 0

        self.total_fuel_consumption = 0
        self.fuel_consumption_count = 0

        # Pedestrian control
        self.is_pedestrian_list = [False]*4         # Whether there is a pedestrian request
        self.pedestrian_time_mark_list = [None]*4      # Planned pedestrian time (In case some cars insterted and interrupt the pedestiran time)

        # Spill-back info
        self.my_road_info = [{'avail_len':cfg.TOTAL_LEN, 'delay':0} for i in range(4*cfg.LANE_NUM_PER_DIRECTION)] # For updating the available road info
        self.others_road_info = [None]*(4*cfg.LANE_NUM_PER_DIRECTION)   # Reference to read others road info
        self.spillback_delay_record = [0]*(4*cfg.LANE_NUM_PER_DIRECTION)

        self.set_round_lane()

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

    def set_round_lane(self):
        for idx in range(1,5):
            for jdx in range(cfg.LANE_NUM_PER_DIRECTION):
                idx_str = self.ID + '_' + str(idx)+'_'+str(jdx)
                self.in_lanes.append(idx_str)

    def check_in_my_region(self, lane_id):
        if lane_id in self.in_lanes:
            return "On my lane"
        else:
            lane_data = lane_id.split("_")
            lane_id_short = lane_data[0] + "_" + lane_data[1]
            if lane_id_short == ":" + self.ID:
                return "In my intersection"
            else:
                return "Not me"

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

    def update_path(self, car_id, car_turn, intersection_dir):
        id_data = self.ID.split('_')
        x_idx = int(id_data[0])
        y_idx = int(id_data[1])

        target_dir = None

        if car_turn == 'R':
            target_dir = ((intersection_dir-1)+1)%4+1
        elif car_turn == 'S':
            target_dir = intersection_dir
        elif car_turn == 'L':
            target_dir = ((intersection_dir-1)-1)%4+1

        if target_dir == 1:
            x_idx = x_idx + 1
            y_idx = y_idx
        elif target_dir == 2:
            x_idx = x_idx
            y_idx = y_idx - 1
        elif target_dir == 3:
            x_idx = x_idx - 1
            y_idx = y_idx
        elif target_dir == 4:
            x_idx = x_idx
            y_idx = y_idx + 1

        intersection_manager_id = "00%i"%(x_idx) + "_" + "00%i"%(y_idx)

        target_edge = intersection_manager_id + "_" + str(target_dir)
        traci.vehicle.changeTarget(car_id, target_edge)
        traci.vehicle.setMaxSpeed(car_id, cfg.MAX_SPEED)
        traci.vehicle.setColor(car_id, (255,255,255))

    def delete_car(self, car_id):
        if car_id in self.car_list:
            self.car_list.pop(car_id)
    def update_car(self, car_id, lane_id, simu_step, car_turn, next_turn):

        if lane_id in self.in_lanes:
            lane_data = lane_id.split("_")
            lane_direction = int(lane_data[2])
            lane_sub_idx = int(lane_data[3])
            lane = int(((4-lane_direction))*cfg.LANE_NUM_PER_DIRECTION + (cfg.LANE_NUM_PER_DIRECTION-lane_sub_idx-1))


            # Add car if the car is not in the list yet
            if car_id not in self.car_list:
                # Gather the information of the new car
                #traci.vehicle.setSpeed(car_id, cfg.MAX_SPEED)
                length = traci.vehicle.getLength(car_id)
                turning = car_turn

                new_car = Car(car_id, length, lane, turning, next_turn)
                new_car.Enter_T = simu_step - (traci.vehicle.getLanePosition(car_id))/cfg.MAX_SPEED
                self.car_list[car_id] = new_car

                traci.vehicle.setMaxSpeed(car_id, cfg.MAX_SPEED)
                traci.vehicle.setSpeed(car_id, cfg.MAX_SPEED)

                self.update_path(car_id, car_turn, lane_direction)

            self.car_list[car_id].turn = car_turn
            self.car_list[car_id].next_turn = next_turn

            # Set the position of each cars
            position = traci.lane.getLength(lane_id) - traci.vehicle.getLanePosition(car_id)
            self.car_list[car_id].setPosition(position)


            if (self.car_list[car_id].zone == None) and (position <= cfg.TOTAL_LEN - self.car_list[car_id].length):
                self.car_list[car_id].zone = "AZ"
                self.car_list[car_id].zone_state = "AZ_not_advised"

            elif (self.car_list[car_id].zone == "AZ") and (position <= cfg.PZ_LEN + cfg.GZ_LEN + cfg.BZ_LEN + cfg.CCZ_LEN):
                self.car_list[car_id].zone = "PZ"
                self.car_list[car_id].zone_state = "PZ_not_set"

            elif (self.car_list[car_id].zone == "PZ") and (position <= cfg.GZ_LEN + cfg.BZ_LEN + cfg.CCZ_LEN):
                self.car_list[car_id].zone = "GZ"
                self.car_list[car_id].zone_state = "not_scheduled"

            elif (self.car_list[car_id].zone == "GZ") and (position <= cfg.BZ_LEN + cfg.CCZ_LEN):
                self.car_list[car_id].zone = "BZ"

            elif (self.car_list[car_id].zone == "BZ") and (position <= cfg.CCZ_LEN):
                self.car_list[car_id].zone = "CCZ"

            self.car_list[car_id].lane = lane


    def advise_lane(self, target_car, sched_cars, scheduling_cars, advising_car):
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


    # Three group of cars in three zones
    def run(self, sched_cars, scheduling_cars, advising_car):






        for c_idx in range(len(n_sched_car)):
            n_sched_car[c_idx].D = None



        others_road_info = copy.deepcopy(self.others_road_info)

        Scheduling(self.lane_advisor, sched_car, n_sched_car, advised_n_sched_car, self.cc_list, self.car_list, self.pedestrian_time_mark_list, self.schedule_period_count, others_road_info, self.spillback_delay_record)



        self.schedule_period_count = 0


        advised_lane = self.lane_advisor.adviseLane(self.car_list[car_id], spillback_lane_advise_avoid)


        for lane_idx in range(4*cfg.LANE_NUM_PER_DIRECTION):
            self.my_road_info[lane_idx]['avail_len'] = cfg.TOTAL_LEN - car_accumulate_len_lane[lane_idx] - cfg.HEADWAY
            self.my_road_info[lane_idx]['delay'] = delay_lane[lane_idx]
            self.my_road_info[lane_idx]['simu_step'] = simu_step
            self.my_road_info[lane_idx]['car_delay_position'] = lane_car_delay_position[lane_idx]





##########################
# Scheduling thread that handles scheduling and update the table for lane advising
def Scheduling(lane_advisor, sched_car, n_sched_car, advised_n_sched_car, cc_list, car_list, pedestrian_time_mark_list, schedule_period_count, others_road_info, spillback_delay_record):

    IcaccPlus(sched_car, n_sched_car, advised_n_sched_car, pedestrian_time_mark_list, others_road_info, spillback_delay_record)

    lane_advisor.updateTableFromCars(n_sched_car, advised_n_sched_car)

    for car in n_sched_car:
        car.zone_state = "scheduled"

    # Update the pedestrian ime list
    for direction in range(4):
        if pedestrian_time_mark_list[direction] != None:
            pedestrian_time_mark_list[direction] -= schedule_period_count
        if pedestrian_time_mark_list[direction] < -cfg.PEDESTRIAN_TIME_GAP:
            pedestrian_time_mark_list[direction] = None
