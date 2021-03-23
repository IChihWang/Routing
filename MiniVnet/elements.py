
import sys
sys.path.append('./Roadrunner')

from IntersectionManager import IntersectionManager

class Intersection_point:
    def __init__(self, coordinate, GZ_BZ_CCZ_len, HEADWAY, TOTAL_LEN):
        self.coordinate = coordinate
        self.id = "%3.3o"%(coordinate[0]) + "_" + "%3.3o"%(coordinate[1])
        self.manager = IntersectionManager(self.id)

        self.GZ_BZ_CCZ_len = GZ_BZ_CCZ_len
        self.AZ_accumulated_size = 0        # Accumulated car length on AZ, PZ
        self.GZ_accumulated_size = 0        # Accumulated car length on GZ, BZ and CCZ
        self.HEADWAY = HEADWAY
        self.TOTAL_LEN = TOTAL_LEN

    def add_sched_car(self, car):
        self.manager.sched_cars[car.id] = car
        self.GZ_accumulated_size += (car.length + self.HEADWAY)
        self.update_my_spillback_info(car)
    def add_scheduling_cars(self, car):
        self.manager.scheduling_cars[car.id] = car
        self.GZ_accumulated_size += (car.length + self.HEADWAY)
        self.update_my_spillback_info(car)
    def add_advising_car(self, car):
        self.manager.advising_car[car.id] = car
        self.AZ_accumulated_size += (car.length + self.HEADWAY)
        self.update_my_spillback_info(car)

    def delete_car_from_database(self, car, type):
        if type == "lane_advising":
            car_in_database = self.manager.advising_car[car.id]
            del self.manager.advising_car[car.id]
            self.update_my_spillback_info(car_in_database)
            self.AZ_accumulated_size -= (car.length + self.HEADWAY)
        elif type == "scheduling":
            car_in_database = self.manager.scheduling_cars[car.id]
            del self.manager.scheduling_cars[car.id]
            self.update_my_spillback_info(car_in_database)
            self.GZ_accumulated_size -= (car.length + self.HEADWAY)
        elif type == "scheduled":
            car_in_database = self.manager.sched_cars[car.id]
            del self.manager.sched_cars[car.id]
            self.update_my_spillback_info(car_in_database)
            self.GZ_accumulated_size -= (car.length + self.HEADWAY)

    def update_my_spillback_info(self, car):
        lane_idx = car.lane
        self.manager.my_road_info[lane_idx]['avail_len'] = self.TOTAL_LEN - (self.GZ_accumulated_size + self.AZ_accumulated_size + self.HEADWAY)

        scheduled_car_list = []
        for car in self.manager.sched_cars.values():
            if car.lane == lane_idx:
                scheduled_car_list.append(car)
        sorted_scheduled_car_list = sorted(scheduled_car_list, key=lambda car: car.OT)

        lane_car_delay_position = []
        car_accumulate_len_lane = 7.5   # cfg.CCZ_DEC2_LEN+cfg.CCZ_ACC_LEN
        for car in sorted_scheduled_car_list:
            car_accumulate_len_lane += car.length + self.HEADWAY
            lane_car_delay_position.append({"position":car_accumulate_len_lane, "delay":car.D})

        self.manager.my_road_info[lane_idx]['car_delay_position'] = lane_car_delay_position
        self.manager.my_road_info[lane_idx]['delay'] = 0
        if len(lane_car_delay_position) > 0:
            self.manager.my_road_info[lane_idx]['delay'] = lane_car_delay_position[-1]["delay"]

    def connect(self, my_direction, intersection_point, its_direction):
        self.manager.connect(my_direction, intersection_point.manager, its_direction)


    def is_GZ_full(self, car, position_at_offset):
        # Tell if the intersection is full and the car have to wait
        if self.GZ_accumulated_size > self.GZ_BZ_CCZ_len:
            return (False, position_at_offset)
        elif position_at_offset > self.GZ_accumulated_size:
            return (True, position_at_offset)
        else:
            return (True, self.GZ_accumulated_size + car.length + self.HEADWAY)


class Car_in_database:
    def __init__(self, id, length):
        self.id = id
        self.lane = None
        self.length = length
        self.current_turn = None
        self.position = None

        # temporary variables for routing
        self.D = None
        self.OT = None
        self.dst_lane = None
        self.dst_lane_changed_to = None
        self.speed_in_intersection = None

        self.is_spillback = False
        self.is_spillback_strict = False

    def copy_car_for_database(self):
        car = Car_in_database(self.id, self.length)
        car.lane = self.lane
        car.current_turn = self.current_turn
        car.position = self.position
        car.D = self.D
        car.OT = self.OT
        car.dst_lane = self.dst_lane
        car.dst_lane_changed_to = self.dst_lane_changed_to
        car.speed_in_intersection = self.speed_in_intersection

        return car

    def update_dst_lane_and_data(self, LANE_NUM_PER_DIRECTION, V_MAX, TURN_SPEED):
        in_direction = self.lane // LANE_NUM_PER_DIRECTION
        out_direction = None
        if self.current_turn == 'S':
            out_direction = (in_direction+2)%4
        elif self.current_turn == 'R':
            out_direction = (in_direction+1)%4
        elif self.current_turn == 'L':
            out_direction = (in_direction-1)%4

        out_sub_lane = (LANE_NUM_PER_DIRECTION-self.lane%LANE_NUM_PER_DIRECTION-1)
        self.dst_lane = int(out_direction*LANE_NUM_PER_DIRECTION + out_sub_lane)     # Destination lane before next lane change
        self.dst_lane_changed_to = int(out_direction*LANE_NUM_PER_DIRECTION + out_sub_lane)  # Destination lane after next lane change


        # Determine the speed in the intersection
        speed_in_intersection = TURN_SPEED
        if self.current_turn == "S":
            speed_in_intersection = V_MAX
        else:
            speed_in_intersection = TURN_SPEED
        self.speed_in_intersection = speed_in_intersection


class Car(Car_in_database):
    def __init__(self, id, length, dst):
        super().__init__(id, length)
        self.dst_coord = dst
        self.records_intersection_in_database = []  # (type, intersection)
        self.path_data = None

        # temporary variables for routing
        self.src_coord = None
        self.direction_of_src_intersection = None
        self.time_offset_step = None
        self.position_at_offset = None

        self.traveling_time = None
        self.dst_node = None    # record the dst node after routing
