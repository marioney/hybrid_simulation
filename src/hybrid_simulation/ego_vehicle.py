import traci
import rospy
from tf_conversions import transformations
from math import degrees, fabs, cos, sin
from gazebo_msgs.srv import GetModelState

PI = 3.1415926535897


class EgoVehicle:

    def __init__(self, ego_vehicle_id, pos_x=0, pos_y=0, yaw=0):
        self.pos_x = pos_x
        self.pos_y = pos_y
        self.angle = yaw

        self.ego_vehicle_id = ego_vehicle_id
        rospy.wait_for_service("/gazebo/get_model_state")
        self.model_state_srv = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        # rospy.Timer(rospy.Duration.from_sec(1), self.read_position_from_gazebo)

    def read_position_from_gazebo(self):

        try:
            gazebo_coordinates = self.model_state_srv(self.ego_vehicle_id, "")
        except rospy.ServiceException as e:
            rospy.logerr("Error receiving gazebo state: %s", e.message)
            gazebo_coordinates = None

        if gazebo_coordinates is not None:
            # self.yaw = zoe_tf.transform protation.
            roll, pitch, yaw = transformations.euler_from_quaternion([gazebo_coordinates.pose.orientation.x,
                                                                      gazebo_coordinates.pose.orientation.y,
                                                                      gazebo_coordinates.pose.orientation.z,
                                                                      gazebo_coordinates.pose.orientation.w])

            self.pos_x = gazebo_coordinates.pose.position.x + 2*cos(yaw)
            self.pos_y = gazebo_coordinates.pose.position.y + 2*sin(yaw)
                        
            rotate = transformations.quaternion_from_euler(0, 0, -PI/2)
            rotate_2 = transformations.quaternion_from_euler(0, 0, 2*PI-yaw)

            angle_rotated = transformations.quaternion_multiply(rotate_2, rotate)
            # print(angle_rotated)
            roll_r, pitch_r, yaw_r = transformations.euler_from_quaternion(angle_rotated)

            self.angle = degrees(yaw_r)+180

            # moveToXY(self, vehID, edgeID, lane, x, y, angle=-1001.0, keepRoute=1)
            traci.vehicle.moveToXY(self.ego_vehicle_id,
                                   traci.vehicle.getRoadID(self.ego_vehicle_id),
                                   traci.vehicle.getLaneIndex(self.ego_vehicle_id),
                                   self.pos_x,
                                   self.pos_y,
                                   self.angle,
                                   2)

            traci.vehicle.setSpeed(self.ego_vehicle_id,
                                   fabs(gazebo_coordinates.twist.linear.x))
            # rospy.loginfo("Zoe Pos: %.2f, %.2f - Yaw: %.2f -  Speed %.2f",
            #               self.pos_x,
            #               self.pos_y,
            #               self.angle,
            #               fabs(gazebo_coordinates.twist.linear.x))
            # print(degrees(self.yaw), degrees(pitch_r), degrees(pitch_r) + 180)

    def init_ego_car_control(self):

        rospy.loginfo("Ego-vehicle departed")
        # rospy.loginfo("EgoCarPosition %.2f - %.2f", self.pos_x, self.pos_y)
        traci.vehicle.setSpeedMode(self.ego_vehicle_id, 0)
        traci.vehicle.setLaneChangeMode(self.ego_vehicle_id, 0)
        traci.vehicle.setSpeed(self.ego_vehicle_id, 0)
        # traci.vehicle.moveToXY(self.ego_vehicle_id,
        #                        traci.vehicle.getRoadID(self.ego_vehicle_id),
        #                        traci.vehicle.getLaneID(self.ego_vehicle_id),
        #                        self.pos_x,
        #                        self.pos_y,
        #                        self.angle,
        #                        2)
