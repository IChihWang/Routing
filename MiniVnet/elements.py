

class Intersection_point:
    def __init__(self, coordinate):
        self.coordinate = coordinate
        self.id = "%3.3o"%(coordinate[0]) + "_" + "%3.3o"%(coordinate[1])
        self.manager = IntersectionManager(self.id)

class Car:
    def __init__(self, id):
        self.id = id

class Car_in_database:
    def __init__(self, id):
        self.id = id
