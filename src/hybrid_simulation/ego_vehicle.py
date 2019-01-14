import traci
import rospy
from tf_conversions import transformations
from hybrid_simulation.msg import ChangeLane
from math import degrees, fabs, cos, sin
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState


PI = 3.1415926535897


class EgoVehicle:

    def __init__(self, ego_vehicle_id, external_control_mode):
        self.resume_simulation = False

        self.external_control_mode = external_control_mode
        self.ego_vehicle_id = ego_vehicle_id
        # rospy.wait_for_service("/gazebo/get_model_state")
        # self.get_model_state_srv = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        # self.set_model_state_srv = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

        self.vehicle_control_sub = rospy.Subscriber('vehicle_change_lane', ChangeLane, self.change_lane_callback)
        self.dmaking_time_step = rospy.get_param('~dmaking_time_step', 3.0)

        self.init_ego_car_control(self.external_control_mode)

    # def init_ego_car_control(self, control_from_gazebo, lane_change=False):
    def init_ego_car_control(self, flag_external_control):

        rospy.loginfo("Ego-vehicle departed")
        # Longitudinal mode
        # all checks off -> Speed Mode = 0
        # disable right of way check -> Speed Mode = 23
        # all checks on -> Speed Mode = 31

        # Lane change mode
        # 1621 - Default change mode, all checks on
        # disable all autonomous changing but still handle safety checks in the simulation,
        # 0   - do not respect other drivers when following TraCI requests, adapt speed to fulfill request
        # 256 - collision avoidance
        # 512 - collision avoidance and safety - gap enforcement

        if flag_external_control == 'longitudinal':
            # external software controls the longitudinal motion
            # no lane changes
            rospy.loginfo("Ego-vehicle: longitudinal control mode")

            # no automatic lane changes and no checks -> Lane Change Mode = 0
            traci.vehicle.setLaneChangeMode(self.ego_vehicle_id, 0)
            # all checks off -> Speed Mode = 0
            traci.vehicle.setSpeedMode(self.ego_vehicle_id, 0)
            # set default speed to max Speed
            traci.vehicle.setSpeed(self.ego_vehicle_id,traci.vehicle.getMaxSpeed(self.ego_vehicle_id))

        elif flag_external_control == 'lateral':
            # external software provides lane change commands
            # SUMO takes care of the longitudinal motion
            rospy.loginfo("Ego-vehicle: lateral control mode")

            # all checks on -> Speed Mode = 31
            traci.vehicle.setSpeedMode(self.ego_vehicle_id, 31)
            # no automatic lane changes and no checks -> Lane Change Mode = 0
            traci.vehicle.setLaneChangeMode(self.ego_vehicle_id, 0)

        else:
            # SUMO controls the lateral and longitudinal motion
            rospy.loginfo("Ego-vehicle: auto control mode")

            # all checks on -> Speed Mode = 31
            traci.vehicle.setSpeedMode(self.ego_vehicle_id, 31)
            # 1621 - Default change mode, all checks on
            traci.vehicle.setLaneChangeMode(self.ego_vehicle_id, 1621)


    def change_lane_callback(self, data):

        # check if it's a new (not zero) command
        if self.resume_simulation is False:
            self.resume_simulation = True
        if data.lane_change != 0:
            direction = 0
            if data.lane_change == 1:
                direction = 1
            elif data.lane_change == -1:
                direction = -1
            # if traci.vehicle.couldChangeLane(self.ego_vehicle_id, direction):
            lane_index = traci.vehicle.getLaneIndex(self.ego_vehicle_id)
            rospy.loginfo("EgoVehicle Lane %d", lane_index)
            lane_index += direction
            rospy.loginfo("EgoVehicle desired Lane %d", lane_index)
            try:
                traci.vehicle.changeLane(self.ego_vehicle_id, lane_index, 0)
            except traci.TraCIException as e:
                rospy.logerr("Error changing lane: %s", e.message)
        # #  Change speed according to acceleration command

        if data.accel != -100:
            traci.vehicle.setSpeedMode(self.ego_vehicle_id, 0)
            vehicle_speed = traci.vehicle.getSpeed(self.ego_vehicle_id)
            desired_speed = vehicle_speed + data.accel * self.dmaking_time_step
            rospy.loginfo("EgoVehicle Speed %.2f", vehicle_speed)
            rospy.loginfo("EgoVehicle desired Speed %.2f", desired_speed)
            try:
                traci.vehicle.setSpeed(self.ego_vehicle_id, desired_speed)
                # traci.vehicle.slowDown(self.ego_vehicle_id, desired_speed, self.dmaking_time_step)
            except traci.TraCIException as e:
                rospy.logerr("Error setting acceleration: %s", e.message)
