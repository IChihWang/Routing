#sys.path.append('./Roadrunner')
#from IntersectionManager import IntersectionManager

class Intersection_point:
    def __init__(self, coordinate, GZ_BZ_CCZ_len, HEADWAY):
        self.coordinate = coordinate
        self.id = "%3.3o"%(coordinate[0]) + "_" + "%3.3o"%(coordinate[1])
        self.manager = IntersectionManager(self.id, V_MAX)
        self.scheduled_car = []
        self.n_scheduled_car = []
        self.GZ_BZ_CCZ_len = GZ_BZ_CCZ_len
        self.GZ_accumulated_size = 0
        self.HEADWAY = HEADWAY

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


class Car(Car_in_database):
    def __init__(self, id, length, dst):
        super().__init__(id, length)
        self.dst_coord = dst

        # temporary variables for routing
        self.src_coord = None
        self.direction_of_src_intersection = None
        self.time_offset_step = None
        self.position_at_offset = None

        self.traveling_time = None
        self.dst_node = None    # record the dst node after routing

    def copy_car_for_database(self):
        car = Car_in_database(self.id, self.length)
        car.lane = self.lane
        car.current_turn = self.current_turn
        car.position = self.position
        car.D = self.D
        car.OT = self.OT

        return car
