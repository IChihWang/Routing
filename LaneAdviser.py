import copy
import random
import config as cfg
from random import randrange
import json
import math
import numpy
#import myGraphic

from get_inter_length_info import Data

inter_length_data = Data()

RESOLUTION = 2  # According to gen_advise.cpp

class LaneAdviser:
    def __init__(self):
            # Number of the boxes in one dimention
            num_di = RESOLUTION*(LANE_NUM_PER_DIRECTION*2+1)

            # 2D matrix of timing for advising
            timeMatrix = [[0]* num_di for i in range(num_di)]

            # Load the information of which boxes a trajectory will affect
            with open('advise_info/advise_info'+str(LANE_NUM_PER_DIRECTION)+'.json', 'r') as file:
                lane_dict = json.load(file)

            # Record the lane advice on each directions && initialize
            all_default_lane = dict()
            for direction in range(4):
                all_default_lane[(direction, 'L')] = direction*LANE_NUM_PER_DIRECTION
                all_default_lane[(direction, 'R')] = direction*LANE_NUM_PER_DIRECTION + (LANE_NUM_PER_DIRECTION -1)
                all_default_lane[(direction, 'S')] = (all_default_lane[(direction, 'L')] + all_default_lane[(direction, 'R')])//2

            count_advised_not_secheduled_car_num = dict()
            for lane_id in range(4*LANE_NUM_PER_DIRECTION):
                count_advised_not_secheduled_car_num[(lane_id, 'L')] = 0
                count_advised_not_secheduled_car_num[(lane_id, 'R')] = 0
                count_advised_not_secheduled_car_num[(lane_id, 'S')] = 0

    def copyMatrix(self):
        return copy.deepcopy(timeMatrix)

    # Update the table (final version for simulation usage)
    def updateTableFromCars(self, n_sched_car, advised_n_sched_car):
        resetTable()

        for car in n_sched_car:
            updateTable(car.lane, car.current_turn, car.OT+car.D, timeMatrix)

        # Case 1: update with the latest car
        '''
        for car in advised_n_sched_car:
            updateTable(car.lane, car.current_turn, car.position/ V_MAX, timeMatrix)
        #'''
        # Case 2: add costs
        '''
        for car in advised_n_sched_car:
            updateTableAfterAdvise(car.lane, car.current_turn, car.length, timeMatrix)
        #'''

        # Case 3: (case 1 + considering the halt) update with the latest car
        '''
        latest_delay_list = dict()
        cars_on_lanes = dict()
        for idx in range(4*LANE_NUM_PER_DIRECTION):
            cars_on_lanes[idx] = []
        for car in n_sched_car:
            cars_on_lanes[car.lane].append(car)
        for idx in range(4*LANE_NUM_PER_DIRECTION):
            sorted_car_list = sorted(cars_on_lanes[idx], key=lambda car: car.position, reverse=true)
            if len(sorted_car_list) > 0:
                latest_delay_list[idx] = sorted_car_list[0].D
            else:
                latest_delay_list[idx] = 0

        # Consider the halting time
        for car in advised_n_sched_car:
            if car.H_should_halt == true:
                updateTable(car.lane, car.current_turn, car.position/ V_MAX+latest_delay_list[idx], timeMatrix)
            else:
                updateTable(car.lane, car.current_turn, car.position/ V_MAX, timeMatrix)
        #'''

        # Case 4: (case 1 + considering the halt) update with the latest car  "Using desired_lane!!!!!!!""

        latest_delay_list = dict()
        cars_on_lanes = dict()
        for idx in range(4*LANE_NUM_PER_DIRECTION):
            cars_on_lanes[idx] = []
        for car in n_sched_car:
            cars_on_lanes[car.desired_lane].append(car)
        for idx in range(4*LANE_NUM_PER_DIRECTION):
            sorted_car_list = sorted(cars_on_lanes[idx], key=lambda car: car.position, reverse=true)
            if len(sorted_car_list) > 0:
                latest_delay_list[idx] = sorted_car_list[0].D
            else:
                latest_delay_list[idx] = 0

        for car in advised_n_sched_car:
            if (car.CC_state != None) &&  ("Platoon" in car.CC_state):
                updateTable(car.lane, car.current_turn, car.position/ V_MAX+latest_delay_list[car.desired_lane], timeMatrix)
            else:
                updateTable(car.lane, car.current_turn, car.position/ V_MAX, timeMatrix)


        # Count car number on each lane of advised but not scheduled
        for lane_id in range(4*LANE_NUM_PER_DIRECTION):
            count_advised_not_secheduled_car_num[(lane_id, 'L')] = 0
            count_advised_not_secheduled_car_num[(lane_id, 'R')] = 0
            count_advised_not_secheduled_car_num[(lane_id, 'S')] = 0
        for car in advised_n_sched_car:
            count_advised_not_secheduled_car_num[(car.lane, car.current_turn)] += 1

        #myGraphic.gui.setTimeMatrix(timeMatrix)
    # Give lane advice to Cars
    def adviseLane(self, car, spillback_lane_advise_avoid):
        advise_lane = None

        # If spillback detected, avoid the other lanes
        if spillback_lane_advise_avoid[car.dst_lane] == true || spillback_lane_advise_avoid[car.dst_lane_changed_to] == true :
            # Get the shortest || the most ideal lane
            start_lane = (car.lane//LANE_NUM_PER_DIRECTION)*LANE_NUM_PER_DIRECTION
            ideal_lane = None
            if car.current_turn == 'R':
                ideal_lane = start_lane+LANE_NUM_PER_DIRECTION-1
            else if car.current_turn == 'L':
                ideal_lane = start_lane
            else:
                ideal_lane = start_lane+(LANE_NUM_PER_DIRECTION//2)
            advise_lane = ideal_lane

            #print("ADV: ", car.ID, advise_lane)
            # The lane is chosen, update the matrix update giving the lane advice
            updateTableAfterAdvise(advise_lane, car.current_turn, car.length, timeMatrix)
            # Record the given lane
            all_default_lane[(car.lane//LANE_NUM_PER_DIRECTION, car.current_turn)] = advise_lane
            # Change the exact index to the lane index of one direction
            advise_lane = advise_lane%LANE_NUM_PER_DIRECTION

            return LANE_NUM_PER_DIRECTION-advise_lane-1 # The index of SUMO is reversed


        # Sort out the LOTs && list the candidates
        start_lane = (car.lane//LANE_NUM_PER_DIRECTION)*LANE_NUM_PER_DIRECTION
        occup_time_list = [getMaxTime(start_lane+idx, car.current_turn, timeMatrix)+inter_length_data.getIntertime(start_lane+idx, car.current_turn) for idx in range(LANE_NUM_PER_DIRECTION)]
        candidate_list = numpy.argsort(occup_time_list)

        # Get the shortest || the most ideal lane
        ideal_lane = None
        if car.current_turn == 'R':
            ideal_lane = start_lane+LANE_NUM_PER_DIRECTION-1
        else if car.current_turn == 'L':
            ideal_lane = start_lane
        else:
            # find one mid-lane with smallest LOTs
            if LANE_NUM_PER_DIRECTION > 2:
                ideal_lane = start_lane+candidate_list[0]
            else:
                for lane_idx in candidate_list:
                    if lane_idx != 0 && lane_idx != LANE_NUM_PER_DIRECTION:
                        ideal_lane = start_lane+lane_idx
                        break
        advise_lane = ideal_lane

        # Scan through the candidates && see if we want to change our candidates
        # Get the cost of the ideal trajectory
        ideal_timeMatrix = copyMatrix()
        updateTableAfterAdvise(ideal_lane, car.current_turn, car.length, ideal_timeMatrix)
        ideal_others_LOT_list = dict()
        for key, lane_i in all_default_lane.items():
            turn_i = key[1]
            ideal_others_LOT_list[key] = getMaxTime(lane_i, turn_i, ideal_timeMatrix)


        for lane_idx in candidate_list:
            candidate_lane = start_lane+lane_idx

            # The ideal lane has the smallest cost so far
            if start_lane+lane_idx == ideal_lane:
                break

            # see if the trajectory affects others
            candidate_timeMatrix = copyMatrix()
            updateTableAfterAdvise(candidate_lane, car.current_turn, car.length, candidate_timeMatrix)
            candidate_others_LOT_list = dict()
            for key, lane_i in all_default_lane.items():
                turn_i = key[1]
                candidate_others_LOT_list[key] = getMaxTime(lane_i, turn_i, candidate_timeMatrix)

            # Compare the LOTs
            is_better = true
            for key, lane_i in all_default_lane.items():
                turn_i = key[1]
                # Skip the candidate lanes, because obviously it will increase
                if lane_i == int(candidate_lane):
                    continue
                else if count_advised_not_secheduled_car_num[(lane_i, turn_i)] > 0:
                    if candidate_others_LOT_list[key] > ideal_others_LOT_list[key]:
                        is_better = false
                        break

            if not is_better:
                continue
            else:
                advise_lane = candidate_lane
                break

        # The lane is chosen, update the matrix update giving the lane advice
        updateTableAfterAdvise(advise_lane, car.current_turn, car.length, timeMatrix)
        # Record the given lane
        all_default_lane[(car.lane//LANE_NUM_PER_DIRECTION, car.current_turn)] = advise_lane
        # Change the exact index to the lane index of one direction
        advise_lane = advise_lane%LANE_NUM_PER_DIRECTION
        #myGraphic.gui.setTimeMatrix(timeMatrix)
        #myGraphic.gui.setAdviseMatrix(all_default_lane)

        return LANE_NUM_PER_DIRECTION-advise_lane-1 # The index of SUMO is reversed


    '''
    # Give lane advice to Cars
    def adviseLane_v2(self, car):
        advise_lane = None

        # Sort out the LOTs && list the candidates
        start_lane = (car.lane//LANE_NUM_PER_DIRECTION)*LANE_NUM_PER_DIRECTION
        occup_time_list = [getMaxTime(start_lane+idx, car.current_turn, timeMatrix) for idx in range(LANE_NUM_PER_DIRECTION)]
        candidate_list = numpy.argsort(occup_time_list)

        # Get the shortest || the most ideal lane
        ideal_lane = None
        if car.current_turn == 'R':
            ideal_lane = start_lane+LANE_NUM_PER_DIRECTION-1
        else if car.current_turn == 'L':
            ideal_lane = start_lane
        else:
            # find one mid-lane with smallest LOTs
            if LANE_NUM_PER_DIRECTION > 2:
                ideal_lane = start_lane+candidate_list[0]
            else:
                for lane_idx in candidate_list:
                    if lane_idx != 0 && lane_idx != LANE_NUM_PER_DIRECTION:
                        ideal_lane = start_lane+lane_idx
                        break
        advise_lane = ideal_lane

        # Scan through the candidates && see if we want to change our candidates
        # Get the cost of the ideal trajectory
        ideal_timeMatrix = copyMatrix()
        updateTableAfterAdvise(ideal_lane, car.current_turn, car.length, ideal_timeMatrix)
        ideal_others_LOT_list = dict()
        for lane_i in range(LANE_NUM_PER_DIRECTION*4):
            for turning_i in ['S', 'R', 'L']:
                ideal_others_LOT_list[(lane_i, turning_i)] = getMaxTime(lane_i, turning_i, ideal_timeMatrix)


        for lane_idx in candidate_list:
            candidate_lane = start_lane+lane_idx

            # The ideal lane has the smallest cost so far
            if start_lane+lane_idx == ideal_lane:
                break

            # see if the trajectory affects others
            candidate_timeMatrix = copyMatrix()
            updateTableAfterAdvise(candidate_lane, car.current_turn, car.length, candidate_timeMatrix)
            candidate_others_LOT_list = dict()
            for lane_i in range(LANE_NUM_PER_DIRECTION*4):
                for turning_i in ['S', 'R', 'L']:
                    candidate_others_LOT_list[(lane_i, turning_i)] = getMaxTime(lane_i, turning_i, candidate_timeMatrix)

            # Compare the LOTs
            is_better = true
            for lane_i in range(LANE_NUM_PER_DIRECTION*4):
                for turning_i in ['S', 'R', 'L']:
                    if lane_i == int(candidate_lane):
                        continue
                    else if count_advised_not_secheduled_car_num[(lane_i, turning_i)] > 0:
                        if candidate_others_LOT_list[(lane_i, turning_i)] > ideal_others_LOT_list[(lane_i, turning_i)]:
                            is_better = false
                            break

            if not is_better:
                continue
            else:
                advise_lane = candidate_lane
                break

        # The lane is chosen, update the matrix update giving the lane advice
        updateTableAfterAdvise(advise_lane, car.current_turn, car.length, timeMatrix)

        # Record the given lane
        all_default_lane[(car.lane//LANE_NUM_PER_DIRECTION, car.current_turn)] = advise_lane

        if (advise_lane//LANE_NUM_PER_DIRECTION!= car.lane//LANE_NUM_PER_DIRECTION):
            print(car.ID, advise_lane, car.lane)
        assert(advise_lane//LANE_NUM_PER_DIRECTION== car.lane//LANE_NUM_PER_DIRECTION)
        # Change the exact index to the lane index of one direction
        advise_lane = advise_lane%LANE_NUM_PER_DIRECTION
        #myGraphic.gui.setTimeMatrix(timeMatrix)
        #myGraphic.gui.setAdviseMatrix(all_default_lane)

        return LANE_NUM_PER_DIRECTION-advise_lane-1 # The index of SUMO is reversed
    '''

    # Give lane advice to Cars
    def adviseLaneShortestTrajectory(self, car):
        advise_lane = None

        # Sort out the LOTs && list the candidates
        start_lane = (car.lane//LANE_NUM_PER_DIRECTION)*LANE_NUM_PER_DIRECTION
        occup_time_list = [getMaxTime(start_lane+idx, car.current_turn, timeMatrix) for idx in range(LANE_NUM_PER_DIRECTION)]
        candidate_list = numpy.argsort(occup_time_list)

        # Get the shortest || the most ideal lane
        ideal_lane = None
        if car.current_turn == 'R':
            ideal_lane = start_lane+LANE_NUM_PER_DIRECTION-1
        else if car.current_turn == 'L':
            ideal_lane = start_lane
        else:
            # find one mid-lane with smallest LOTs
            if LANE_NUM_PER_DIRECTION > 2:
                ideal_lane = start_lane+candidate_list[0]
            else:
                for lane_idx in candidate_list:
                    if lane_idx != 0 && lane_idx != LANE_NUM_PER_DIRECTION:
                        ideal_lane = start_lane+lane_idx
                        break
        advise_lane = ideal_lane# Change the exact index to the lane index of one direction
        advise_lane = advise_lane%LANE_NUM_PER_DIRECTION

        #myGraphic.gui.setTimeMatrix([[0]* num_di for i in range(num_di)])
        #myGraphic.gui.setAdviseMatrix(all_default_lane)

        return LANE_NUM_PER_DIRECTION-advise_lane-1 # The index of SUMO is reversed


    def getMaxTime(self, lane, direction, timeMatrix):
        # get the earliest time when a car can drive
        # through lane "lane" in direction "direction".

        # lane : int
        # direction : string

        temp = []
        quotient = lane // LANE_NUM_PER_DIRECTION

        if quotient == 0:
            # no rotation
            for e in lane_dict[str(lane % LANE_NUM_PER_DIRECTION) + direction]:
                temp.append(timeMatrix[e['X']][e['Y']])

        else if quotient == 1:
            # rotate 90 degree clockwise
            for e in lane_dict[str(lane % LANE_NUM_PER_DIRECTION) + direction]:
                temp.append(timeMatrix[num_di-1 - e['Y']][e['X']])

        else if quotient == 2:
            # rotate 180 degree clockwise
            for e in lane_dict[str(lane % LANE_NUM_PER_DIRECTION) + direction]:
                temp.append(timeMatrix[num_di-1 - e['X']][num_di-1 - e['Y']])

        else:  # quotient == 4
            # rotate 270 degree clockwise
            for e in lane_dict[str(lane % LANE_NUM_PER_DIRECTION) + direction]:
                temp.append(timeMatrix[e['Y']][num_di-1 - e['X']])

        return max(temp)



    def updateTable(self, lane, direction, time, timeMatrix):
        # Update all the squares on the trajectory "lane + direction" to "time".
        quotient = (lane) // LANE_NUM_PER_DIRECTION

        if quotient == 0:
            # no rotation
            for e in lane_dict[str(lane % LANE_NUM_PER_DIRECTION) + direction]:
                if (time > timeMatrix[e['X']][e['Y']]):
                    timeMatrix[e['X']][e['Y']] = time


        else if quotient == 1:
            # rotate 90 degree clockwise
            for e in lane_dict[str(lane % LANE_NUM_PER_DIRECTION) + direction]:
                if (time > timeMatrix[num_di-1 - e['Y']][e['X']]):
                    timeMatrix[num_di-1 - e['Y']][e['X']] = time


        else if quotient == 2:
            # rotate 180 degree clockwise
            for e in lane_dict[str(lane % LANE_NUM_PER_DIRECTION) + direction]:
                if (time > timeMatrix[num_di-1 - e['X']][num_di-1 - e['Y']]):
                    timeMatrix[num_di-1 - e['X']][num_di-1 - e['Y']] = time


        else:  # quotient == 3
            # rotate 270 degree clockwise
            for e in lane_dict[str(lane % LANE_NUM_PER_DIRECTION) + direction]:
                if (time > timeMatrix[e['Y']][num_di-1 - e['X']]):
                    timeMatrix[e['Y']][num_di-1 - e['X']] = time


    def updateTableAfterAdvise(self, lane, direction, car_length, timeMatrix):

        # Update all the squares on the trajectory "lane + direction" to "diff_time".

        quotient = lane // LANE_NUM_PER_DIRECTION
        speed = None

        if direction == 'S':
            speed =  V_MAX
        else:
            speed = TURN_SPEED

        if quotient == 0:
            # lane 0 || 1, no rotation
            for e in lane_dict[str(lane % LANE_NUM_PER_DIRECTION) + direction]:
                time = e['distance']*LANE_WIDTH/speed + car_length/speed
                timeMatrix[e['X']][e['Y']] += time

        else if quotient == 1:
            # lane 2 || 3, rotate 90 degree clockwise
            for e in lane_dict[str(lane % LANE_NUM_PER_DIRECTION) + direction]:
                time = e['distance']*LANE_WIDTH/speed + car_length/speed
                timeMatrix[num_di-1 - e['Y']][e['X']] += time

        else if quotient == 2:
            # lane 4 || 5, rotate 180 degree clockwise
            for e in lane_dict[str(lane % LANE_NUM_PER_DIRECTION) + direction]:
                time = e['distance']*LANE_WIDTH/speed + car_length/speed
                timeMatrix[num_di-1 - e['X']][num_di-1 - e['Y']] += time

        else:  # quotient == 3
            # lane 6 || 7, rotate 270 degree clockwise
            for e in lane_dict[str(lane % LANE_NUM_PER_DIRECTION) + direction]:
                time = e['distance']*LANE_WIDTH/speed + car_length/speed
                timeMatrix[e['Y']][num_di-1 - e['X']] += time


    # Reset the records
    def resetTable(self):
        timeMatrix = [[0] * num_di for i in range(num_di)]
