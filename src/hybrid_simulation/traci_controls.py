import traci


class Status:

    def __init__(self, edge, pos):
        self.edge = edge
        self.pos = pos
        self.broken = False
        self.target = None
        self.target_pos = None
        self.delay = None

    def __repr__(self):
        return "%s,%s" % (self.edge, self.pos)


class Setting:
    def __init__(self, step=0, verbose=False):
        self.step = step
        self.verbose = verbose


class TraciControls:

    def __init__(self, vehicle_status={}):
        self.vehicle_status = vehicle_status
        self.setting = Setting()

    def stop_at(self, vehicle_id, edge, pos=None):
        if pos is None:
            pos = 1.0
        traci.vehicle.changeTarget(vehicle_id, edge)
        if self.setting.verbose:
            print("stop_at: ", vehicle_id, edge, pos)
    #        print vehicleStatus[vehicleID]
    #        print traci.vehicle.getRoute(vehicleID)
        traci.vehicle.setStop(vehicle_id, edge, pos)
        self.vehicle_status[vehicle_id].target = edge
        self.vehicle_status[vehicle_id].target_pos = pos
        self.vehicle_status[vehicle_id].broken = True

    def restart(self, vehicle_id, new_target=None, delay=0):
        v = self.vehicle_status[vehicle_id]
        if new_target:
            print("new dest", vehicle_id, new_target)
            traci.vehicle.changeTarget(vehicle_id, new_target)
        print("set stop", vehicle_id, new_target, v.target, v.target_pos, 0, delay)
        traci.vehicle.setStop(vehicle_id, v.target, v.target_pos, 0, delay)
        traci.vehicle.setSpeed(vehicle_id, 1.0)
        v.target = None
        v.target_pos = None
        v.broken = True

    def check_initial_position(self, vehicle_id, edge, pos):
        if vehicle_id in self.vehicle_status:
            self.vehicle_status[vehicle_id].edge = edge
            self.vehicle_status[vehicle_id].pos = pos
        else:
            self.vehicle_status[vehicle_id] = Status(edge, pos)
            if vehicle_id.startswith("broken"):
                self.stop_at(vehicle_id, "L1")


def check_collision(vehicle_id, distance_to_leader=20, safety_distance=0.5):

    min_gap = traci.vehicle.getLeader(vehicle_id, distance_to_leader)
    if min_gap is not None:
        [leader, dist] = min_gap
        # print("Leader: %s", leader)
        # print("Dist: %.2f", dist)
        if dist < (-1*safety_distance):
            print("Vehicle: %s In Collision", vehicle_id)
            return True
    return False
