
from Heuristic.Parameter import *



class Truck():
    def __init__(self, rep_id):
        self.rep_id = rep_id
        self.delivery_time = TRUCK_DELIVERY_TIME
        self.launch_time = LAUNCH_TIME
        self.retrival_time = RETRIVAL_TIME

    def __repr__(self):
        return f"Truck-{self.rep_id}"


class UAV():
    def __init__(self, rep_id):
        self.rep_id = rep_id
        self.endurance = ENDURANCE
        self.velocity = UAV_MAX_VEL
        self.delivery_time = UAV_DELIVERY_TIME

    def calculate_flight_time(self, src, des):
        return src.distanceTo(des) / self.velocity


    def __repr__(self):
        return f"UAV-{self.rep_id}"


