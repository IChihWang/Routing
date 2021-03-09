

class Intersection_point:
    def __init__(self, coordinate):
        self.coordinate = coordinate
        self.id = "%3.3o"%(coordinate[0]) + "_" + "%3.3o"%(coordinate[1])
        self.manager = IntersectionManager(self.id)

class Car:
    def __init__(self, id, length, dst):
        self.id = id
        self.length = length
        self.dst_coord = dst

        # temporary variables for routing
        self.src_coord = None
        self.direction_of_src_intersection = None
        self.time_offset_step = None
        self.position_at_offset = None

class Car_in_database:
    def __init__(self, id):
        self.id = id
