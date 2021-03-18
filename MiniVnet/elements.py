
import sys
sys.path.append('./Roadrunner')

from IntersectionManager import IntersectionManager

class Intersection_point:
    def __init__(self, coordinate, GZ_BZ_CCZ_len, HEADWAY):
        self.coordinate = coordinate
        self.id = "%3.3o"%(coordinate[0]) + "_" + "%3.3o"%(coordinate[1])
        self.manager = IntersectionManager(self.id)

        self.GZ_BZ_CCZ_len = GZ_BZ_CCZ_len
        self.GZ_accumulated_size = 0
        self.HEADWAY = HEADWAY

    def add_sched_car(self, car):
        self.manager.sched_cars[car.id] = car
    def add_scheduling_cars(self, car):
        self.manager.scheduling_cars[car.id] = car
    def add_advising_car(self, car):
        self.manager.advising_car[car.id] = car

    def delete_car_from_database(self, car, type):
        if type == "lane_advising":
            del self.manager.advising_car[car.id]
        elif type == "scheduling":
            del self.manager.scheduling_cars[car.id]
        elif type == "scheduled":
            del self.manager.sched_cars[car.id]


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

    def copy_car_for_database(self):
        car = Car_in_database(self.id, self.length)
        car.lane = self.lane
        car.current_turn = self.current_turn
        car.position = self.position
        car.D = self.D
        car.OT = self.OT
        car.dst_lane = self.dst_lane
        car.dst_lane_changed_to = self.dst_lane_changed_to

        return car

    def update_dst_lane(self, LANE_NUM_PER_DIRECTION):
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
