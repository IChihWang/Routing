
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
        ID = id
        az_list = dict()
        pz_list = dict()

        car_list = dict()   # Cars that needs to be handled
        cc_list = dict()    # Cars under Cruse Control in CCZ
        leaving_cars = dict()   # Cars just entered the intersection (leave the CC zone)

        schedule_period_count = 0
        lane_advisor = LaneAdviser()
        scheduling_thread = None
        in_lanes = []
        out_lanes = []

        # For front car
        CC_last_cars_on_lanes = dict()
        for idx in range(4*LANE_NUM_PER_DIRECTION):
            CC_last_cars_on_lanes[idx] = None


        # Statistics
        total_delays = 0
        total_delays_by_sche = 0
        car_num = 0

        total_fuel_consumption = 0
        fuel_consumption_count = 0

        # Pedestrian control
        is_pedestrian_list = [false]*4         # Whether there is a pedestrian request
        pedestrian_time_mark_list = [None]*4      # Planned pedestrian time (In case some cars insterted  && interrupt the pedestiran time)

        # Spill-back info
        my_road_info = [{'avail_len':TOTAL_LEN, 'delay':0} for i in range(4*LANE_NUM_PER_DIRECTION)] # For updating the available road info
        others_road_info = [None]*(4*LANE_NUM_PER_DIRECTION)   # Reference to read others road info
        spillback_delay_record = [0]*(4*LANE_NUM_PER_DIRECTION)

        set_round_lane()

    def connect(self, my_direction, intersection, its_direction):
        '''
                2
            3   i   1
                0
        ''' #(Lane index are all counter-clockwise)
        for lane_idx in range(LANE_NUM_PER_DIRECTION):
            my_lane = my_direction*LANE_NUM_PER_DIRECTION+lane_idx
            its_lane = its_direction*LANE_NUM_PER_DIRECTION+(LANE_NUM_PER_DIRECTION-lane_idx-1)

            others_road_info[my_lane] = intersection.my_road_info[its_lane]
            intersection.others_road_info[its_lane] = my_road_info[my_lane]

    def set_round_lane(self):
        for idx in range(1,5):
            for jdx in range(LANE_NUM_PER_DIRECTION):
                idx_str = ID + '_' + str(idx)+'_'+str(jdx)
                in_lanes.append(idx_str)

    def check_in_my_region(self, lane_id):
        if lane_id in in_lanes:
            return "On my lane"
        else:
            lane_data = lane_id.split("_")
            lane_id_short = lane_data[0] + "_" + lane_data[1]
            if lane_id_short == ":" + ID:
                return "In my intersection"
            else:
                return "Not me"

    def get_car_info_for_route(self, car_id):
        time_offset = None
        src_intersection_id = None              # After offset, which intersection/node
        direction_of_src_intersection = None
        src_shift_num = 0
        position = car_list[car_id].position

        # Not yet enter Roadrunner
        if car_list[car_id].zone == None:
            diff_pos = position - TOTAL_LEN
            time_offset = diff_pos/MAX_SPEED

            if time_offset > ROUTING_PERIOD:
                # Get the origin intersection id
                lane = car_list[car_id].lane
                direction = lane//LANE_NUM_PER_DIRECTION

                direction_of_src_intersection = direction
                src_intersection_id = ID

            else:
                time_offset = None

        # 1. Not yet but about to enter the intersection region
        # 2. Already inside the intersection region
        if time_offset == None:
            src_shift_num = 1

            # Find lane of the car
            lane = None
            if car_list[car_id].zone == "AZ":
                lane = car_list[car_id].desired_lane
            else:
                lane = car_list[car_id].lane

            # Compute the time_offset
            if not isinstance(car_list[car_id].D, float):
                # Delay is not computed yet
                time_offset = position/MAX_SPEED
                # Estimate by borrowing the known delay
                if len(my_road_info[lane]['car_delay_position']) > 0:
                    time_offset += my_road_info[lane]['car_delay_position'][-1]['delay']
            else:
                # Compute with the known delay
                time_offset = car_list[car_id].OT + car_list[car_id].D

            # Add the time in the intersection
            turning = car_list[car_id].current_turn
            time_in_inter = inter_length_data.getIntertime(lane, turning)
            time_offset += time_in_inter  # time passing intersection

            # Start from next intersection
            if time_offset <= ROUTING_PERIOD:
                # About to exit the intersection
                # Not allowing scheduling for a while
                return None
            else:
                direction = lane//LANE_NUM_PER_DIRECTION

                if turning == 'S':
                    direction_of_src_intersection = direction
                else if turning == 'L':
                    direction_of_src_intersection = (direction+1)%4
                else if turning == 'R':
                    direction_of_src_intersection = (direction-1)%4

                intersection_idx_list = ID.split("_")
                if direction_of_src_intersection == 0:
                    intersection_idx_list[1] = "%3.3o"%(int(intersection_idx_list[1])+1)
                else if direction_of_src_intersection == 1:
                    intersection_idx_list[0] = "%3.3o"%(int(intersection_idx_list[0])-1)
                else if direction_of_src_intersection == 2:
                    intersection_idx_list[1] = "%3.3o"%(int(intersection_idx_list[1])-1)
                else if direction_of_src_intersection == 3:
                    intersection_idx_list[0] = "%3.3o"%(int(intersection_idx_list[0])+1)

                src_intersection_id = intersection_idx_list[0] + "_" + intersection_idx_list[1]

        time_offset_step = int(time_offset//SCHEDULING_PERIOD)
        time_offset = (time_offset_step+1)*SCHEDULING_PERIOD - time_offset
        position_at_offset = TOTAL_LEN - time_offset*MAX_SPEED

        # Might return None as well for temporary forbid of routing
        return (position_at_offset, time_offset_step, src_intersection_id, direction_of_src_intersection, src_shift_num)

    def update_path(self, car_id, car_turn, next_turn, intersection_dir):
        id_data = ID.split('_')
        x_idx = int(id_data[0])
        y_idx = int(id_data[1])

        target_dir = None

        if car_turn == 'R':
            target_dir = ((intersection_dir-1)+1)%4+1
        else if car_turn == 'S':
            target_dir = intersection_dir
        else if car_turn == 'L':
            target_dir = ((intersection_dir-1)-1)%4+1

        if target_dir == 1:
            x_idx = x_idx + 1
            y_idx = y_idx
        else if target_dir == 2:
            x_idx = x_idx
            y_idx = y_idx - 1
        else if target_dir == 3:
            x_idx = x_idx - 1
            y_idx = y_idx
        else if target_dir == 4:
            x_idx = x_idx
            y_idx = y_idx + 1

        intersection_manager_id = "00%i"%(x_idx) + "_" + "00%i"%(y_idx)

        target_edge = intersection_manager_id + "_" + str(target_dir)
        traci.vehicle.changeTarget(car_id, target_edge)
        traci.vehicle.setMaxSpeed(car_id, MAX_SPEED)
        traci.vehicle.setColor(car_id, (255,255,255))

        # Update dst lanes  && info
        car_list[car_id].set_turning(car_turn, next_turn)

    def delete_car(self, car_id):
        if car_id in car_list:
            car_list.pop(car_id)
    def update_car(self, car_id, lane_id, simu_step, car_turn, next_turn):

        if lane_id in in_lanes:
            lane_data = lane_id.split("_")
            lane_direction = int(lane_data[2])
            lane_sub_idx = int(lane_data[3])
            lane = int(((4-lane_direction))*LANE_NUM_PER_DIRECTION + (LANE_NUM_PER_DIRECTION-lane_sub_idx-1))


            # Add car if the car is not in the list yet
            if car_id not in car_list:
                # Gather the information of the new car
                #traci.vehicle.setSpeed(car_id, MAX_SPEED)
                length = traci.vehicle.getLength(car_id)
                turning = car_turn

                new_car = Car(car_id, length, lane, turning, next_turn)
                new_car.Enter_T = simu_step - (traci.vehicle.getLanePosition(car_id))/MAX_SPEED
                car_list[car_id] = new_car

                traci.vehicle.setMaxSpeed(car_id, MAX_SPEED)
                traci.vehicle.setSpeed(car_id, MAX_SPEED)


            update_path(car_id, car_turn, next_turn, lane_direction)
            car_list[car_id].current_turn = car_turn
            car_list[car_id].next_turn = next_turn

            # Set the position of each cars
            position = traci.lane.getLength(lane_id) - traci.vehicle.getLanePosition(car_id)
            car_list[car_id].setPosition(position)


            if (car_list[car_id].zone == None)  && (position <= TOTAL_LEN - car_list[car_id].length):
                car_list[car_id].zone = "AZ"
                car_list[car_id].zone_state = "AZ_not_advised"

            else if (car_list[car_id].zone == "AZ")  && (position <= PZ_LEN + GZ_LEN + BZ_LEN + CCZ_LEN):
                car_list[car_id].zone = "PZ"
                car_list[car_id].zone_state = "PZ_not_set"

            else if (car_list[car_id].zone == "PZ")  && (position <= GZ_LEN + BZ_LEN + CCZ_LEN):
                car_list[car_id].zone = "GZ"
                car_list[car_id].zone_state = "not_scheduled"

            else if (car_list[car_id].zone == "GZ")  && (position <= BZ_LEN + CCZ_LEN):
                car_list[car_id].zone = "BZ"

            else if (car_list[car_id].zone == "BZ")  && (position <= CCZ_LEN):
                car_list[car_id].zone = "CCZ"

            car_list[car_id].lane = lane





    def run(self, simu_step):

        # ===== Update the time OT =====
        for car_key in car_list:
            # Update when the car is scheduled
            if car_list[car_key].OT != None:
                car_list[car_key].OT -= TIME_STEP

            total_fuel_consumption += traci.vehicle.getFuelConsumption(car_key)*TIME_STEP
            fuel_consumption_count += 1


            if isinstance(car_list[car_key].D, float):
                traci.vehicle.setColor(car_key, (100,250,92))
            if car_list[car_key].is_spillback == true:
                traci.vehicle.setColor(car_key, (255,153,51))
            else if car_list[car_key].is_spillback_strict == true:
                traci.vehicle.setColor(car_key, (255,59,59))


        # ===== Entering the intersection (Record the cars) =====
        for car_id, car in car_list.items():
            lane_id = traci.vehicle.getLaneID(car_id)
            if lane_id not in in_lanes:
                traci.vehicle.setSpeed(car_id, car.speed_in_intersection)

                #  leaving_cars[car_id] = car_list[car_id]
                car_list[car_id].Leave_T = simu_step
                total_delays += (car.Leave_T - car.Enter_T) - ((CCZ_LEN+GZ_LEN+BZ_LEN+PZ_LEN+AZ_LEN)/MAX_SPEED)

                # Measurement
                total_delays_by_sche += car.D
                car_num += 1

                car.zone = "Intersection"


        # ===== Starting Cruise control
        to_be_deleted = []
        for car_id, car in pz_list.items():
            if car.position <= CCZ_LEN  && isinstance(car.D, float):

                to_be_deleted.append(car_id)

                if (car.CC_state == "Preseting_done"):
                    car.CC_state = "CruiseControl_ready"
            else if car.position <= CCZ_LEN:
                to_be_deleted.append(car_id)

                if (car.CC_state == None)   || (not ("Platoon" in car.CC_state   || "Entering" in car.CC_state)):
                    car.CC_state = "Keep_Max_speed"

        for car_id in to_be_deleted:
            del pz_list[car_id]




        ##############################################
        # Grouping the cars  && schedule
        # Put here due to the thread handling
        schedule_period_count += TIME_STEP
        if schedule_period_count > GZ_LEN/MAX_SPEED -1:
            if scheduling_thread == None   || (not scheduling_thread.is_alive()):

                # Classify the cars for scheduler
                sched_car = []
                n_sched_car = []
                advised_n_sched_car = []


                for car_id, car in car_list.items():
                    if car.zone == "GZ"   || car.zone == "BZ"   || car.zone == "CCZ":
                        if car.zone_state == "not_scheduled":
                            n_sched_car.append(car)
                        else:
                            sched_car.append(car)
                            traci.vehicle.setColor(car_id, (100,250,92))
                    else if car.zone == "PZ"   || car.zone == "AZ":
                        advised_n_sched_car.append(car)




                for c_idx in range(len(n_sched_car)):
                    n_sched_car[c_idx].D = None

                # Setting the pedestrian list
                is_pedestrian_list = [true]*4
                for direction in range(4):
                    # Cancel the request if a pedestrian time has been scheduled
                    if is_pedestrian_list[direction] == true  && pedestrian_time_mark_list[direction] != None:
                        is_pedestrian_list[direction] = false
                pedestrian_time_mark_list = get_max_AT_direction(sched_car, is_pedestrian_list, pedestrian_time_mark_list)


                others_road_info = copy.deepcopy(others_road_info)

                # scheduling_thread = threading.Thread(target = Scheduling, args = (lane_advisor, sched_car, n_sched_car, advised_n_sched_car, cc_list, car_list, pedestrian_time_mark_list, schedule_period_count, others_road_info, spillback_delay_record))
                # scheduling_thread.start()
                Scheduling(lane_advisor, sched_car, n_sched_car, advised_n_sched_car, cc_list, car_list, pedestrian_time_mark_list, schedule_period_count, others_road_info, spillback_delay_record)


                schedule_period_count = 0

            else:
                print("Warning: the update period does not sync with the length of GZ")






        ################################################
        # Set Max Speed in PZ
        to_be_deleted = []
        for car_id, car in az_list.items():
            if car.zone == "PZ"  && car.zone_state == "PZ_not_set":
                traci.vehicle.setMinGap(car_id, HEADWAY)
                pz_list[car_id] = car
                to_be_deleted.append(car_id)

                # Take over the speed control from the car
                traci.vehicle.setSpeedMode(car_id, 0)

                if car.CC_state == None:
                    car.CC_state = "Preseting_start"

                # Cancel the auto gap
                traci.vehicle.setLaneChangeMode(car_id, 0)

                lane = car.lane
                car.desired_lane = car.lane

                lane_sub_idx = (LANE_NUM_PER_DIRECTION-lane%LANE_NUM_PER_DIRECTION-1)

                # Stay on its lane
                traci.vehicle.changeLane(car_id, lane_sub_idx, 1.0)


                car.zone_state = "PZ_set"
        for car_id in to_be_deleted:
            del az_list[car_id]

        # Set Max Speed in PZ
        for car_id, car in pz_list.items():
            if car.zone == "PZ"  && car.zone_state == "PZ_set":
                lane = car.lane
                car.desired_lane = car.lane

                out_sub_lane = (LANE_NUM_PER_DIRECTION-lane%LANE_NUM_PER_DIRECTION-1)
                car.dst_lane = int(car.out_direction*LANE_NUM_PER_DIRECTION + out_sub_lane)     # Destination lane before next lane change


                # Stay on its lane
                traci.vehicle.changeLane(car_id, out_sub_lane, 1.0)


        ##########################################
        # Cruse Control

        # Start to let cars control itself once it enters the CCZ
        # Each car perform their own Cruise Control behavior
        #ccontrol_list = pz_list.copy()
        #sorted_ccontrol_list = sorted(ccontrol_list.items(), key=lambda x: x[1].position)
        sorted_ccontrol_list = sorted(car_list.items(), key=lambda x: x[1].position)
        # SUPER IMPORTANT: sorted to ensure the following car speed
        for car_id, car in sorted_ccontrol_list:

            # Cars perform their own CC
            if car.zone != None:
                car.handle_CC_behavior(car_list)



        ################################################
        # Change lane in AZ

        # Check whether there is a spillback
        accumulate_car_len_lane = [0]*(4*LANE_NUM_PER_DIRECTION)
        spillback_lane_advise_avoid = [false]*(4*LANE_NUM_PER_DIRECTION)
        #'''
        for car_id, car in car_list.items():
            lane_idx = car.dst_lane
            lane_changed_to_idx = car.dst_lane_changed_to

            if others_road_info[lane_idx] != None:
                accumulate_car_len_lane[lane_idx] += (car.length + HEADWAY)
            if others_road_info[lane_changed_to_idx] != None:
                accumulate_car_len_lane[lane_changed_to_idx] += (car.length + HEADWAY)
            if car.is_spillback == true:
                spillback_lane_advise_avoid[lane_idx] = true

        for lane_idx in range(4*LANE_NUM_PER_DIRECTION):
            if others_road_info[lane_idx] != None:
                if accumulate_car_len_lane[lane_idx] >= others_road_info[lane_idx]['avail_len']:
                    spillback_lane_advise_avoid[lane_idx] = true



        for car_id, car in car_list.items():
            if car.zone == "AZ"  && car.zone_state == "AZ_not_advised":
                az_list[car_id] = car

                traci.vehicle.setMinGap(car_id, 3)
                #traci.vehicle.setLaneChangeMode(car_id, 256)
                traci.vehicle.setLaneChangeMode(car_id, 784)
                #traci.vehicle.setLaneChangeMode(car_id, 528)
                #traci.vehicle.setLaneChangeMode(car_id, 800)
                #traci.vehicle.setLaneChangeMode(car_id, 544)
                # 256 (collision avoidance)   || 512 (collision avoidance  && safety-gap enforcement)

                #time_in_AZ = AZ_LEN/MAX_SPEED *3
                time_in_AZ = 9999.91


                #advised_lane = lane_advisor.adviseLaneShortestTrajectory(car)
                advised_lane = lane_advisor.adviseLane(car_list[car_id], spillback_lane_advise_avoid)
                #advised_lane = lane_advisor.adviseLane_v2(car_list[car_id])
                #advised_lane = random.randrange(0, LANE_NUM_PER_DIRECTION)

                traci.vehicle.changeLane(car_id, advised_lane, time_in_AZ)
                car.desired_lane = int((LANE_NUM_PER_DIRECTION-advised_lane-1)+(car.lane//LANE_NUM_PER_DIRECTION)*LANE_NUM_PER_DIRECTION)
                #car_list[car_id].desired_lane = car.lane

                car.zone_state = "AZ_advised"

            else if car.zone == "AZ"  && car.zone_state == "AZ_advised"  && car.position <= PZ_LEN + GZ_LEN + BZ_LEN + CCZ_LEN + CCZ_ACC_LEN:
                leader_tuple = traci.vehicle.getLeader(car.ID)
                if leader_tuple != None:
                    if leader_tuple[0] in car_list.keys():
                        front_car_ID = leader_tuple[0]
                        front_car = car_list[front_car_ID]
                        front_distance = leader_tuple[1]

                        my_speed = traci.vehicle.getSpeed(car.ID)
                        front_speed = traci.vehicle.getSpeed(front_car.ID)
                        min_catch_up_time = (my_speed-front_speed)/MAX_ACC
                        min_distance = (my_speed-front_speed)*min_catch_up_time

                        min_gap = max(HEADWAY, min_distance+HEADWAY)
                        traci.vehicle.setMinGap(car_id, min_gap)


        ##########################################
        # Update the road info after actions
        car_accumulate_len_lane = [(CCZ_DEC2_LEN+CCZ_ACC_LEN)]*(LANE_NUM_PER_DIRECTION*4)
        delay_lane = [0]*(LANE_NUM_PER_DIRECTION*4)
        car_position_with_delay_lane = [0]*(LANE_NUM_PER_DIRECTION*4)
        lane_car_delay_position = [[] for i in range(LANE_NUM_PER_DIRECTION*4)]

        car_list_sorted = sorted(car_list.values(), key=lambda x: x.position)
        for car in car_list_sorted:
            lane = car.lane
            car_accumulate_len_lane[lane] += car.length + HEADWAY
            if isinstance(car.D, float):
                lane_car_delay_position[lane].append({"position":car_accumulate_len_lane[lane], "delay":car.D})

            if car.position > TOTAL_LEN - AZ_LEN  && lane != car.desired_lane:
                car_accumulate_len_lane[car.desired_lane] += car.length + HEADWAY

            if car.position > car_position_with_delay_lane[lane]  && isinstance(car.D, float):
                car_position_with_delay_lane[lane] = car.position
                #delay_lane[lane] = (car.OT+car.D)-(car.position/MAX_SPEED)
                delay_lane[lane] = car.D


        for lane_idx in range(4*LANE_NUM_PER_DIRECTION):
            my_road_info[lane_idx]['avail_len'] = TOTAL_LEN - car_accumulate_len_lane[lane_idx] - HEADWAY
            my_road_info[lane_idx]['delay'] = delay_lane[lane_idx]
            my_road_info[lane_idx]['simu_step'] = simu_step
            my_road_info[lane_idx]['car_delay_position'] = lane_car_delay_position[lane_idx]


    # Compute teh max AT for pedestrian time
    def get_max_AT_direction(self, sched_car, is_pedestrian_list, pedestrian_time_mark_list):
        max_AT = [0]*4  # Four directions

        for car in sched_car:
            # Compute the direction of entering/exiting for the intersection
            in_dir = car.in_direction
            out_dir = car.out_direction

            # Compute arrival time at the exiting point  && entering point
            in_AT = car.OT + car.D + car.length/car.speed_in_intersection
            out_AT = car.OT + car.D + car.length/MAX_SPEED + inter_length_data.getIntertime(car.lane, car.current_turn)

            # Find max  && update
            if in_AT > max_AT[in_dir]:
                max_AT[in_dir] = in_AT
            if out_AT > max_AT[out_dir]:
                max_AT[out_dir] = out_AT

        for direction in range(4):
            if pedestrian_time_mark_list[direction] != None:
                max_AT[direction] = pedestrian_time_mark_list[direction]
            else if is_pedestrian_list[direction] == false:
                max_AT[direction] = None

        return max_AT




##########################
# Scheduling thread that handles scheduling  && update the table for lane advising
def Scheduling(lane_advisor, sched_car, n_sched_car, advised_n_sched_car, cc_list, car_list, pedestrian_time_mark_list, schedule_period_count, others_road_info, spillback_delay_record):

    if int(sys.argv[3]) == 0:
        IcaccPlus(sched_car, n_sched_car, advised_n_sched_car, pedestrian_time_mark_list, others_road_info, spillback_delay_record)
    else if int(sys.argv[3]) == 1:
        Icacc(sched_car, n_sched_car)
    else if int(sys.argv[3]) == 2:
        Fcfs(sched_car, n_sched_car)

    lane_advisor.updateTableFromCars(n_sched_car, advised_n_sched_car)

    for car in n_sched_car:
        car.zone_state = "scheduled"


    # Update the pedestrian ime list
    for direction in range(4):
        if pedestrian_time_mark_list[direction] != None:
            pedestrian_time_mark_list[direction] -= schedule_period_count
        if pedestrian_time_mark_list[direction] < -PEDESTRIAN_TIME_GAP:
            pedestrian_time_mark_list[direction] = None
