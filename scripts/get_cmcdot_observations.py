#!/usr/bin/env python
"""
Follow a ego vehicle in carla using pure pursuit algorithm
author: Anshul paigwar

To detect if two object would collide read the link below
https://gamedev.stackexchange.com/questions/97337/detect-if-two-objects-are-going-to-collide
https://cocalc.com/projects/9288750d-aa44-43a9-8416-730920bddba8/files/Detect%20collision%20of%20to%20moving%20circles.sagews?session=default
https://codepen.io/sveinatle/pen/OPqLKE?editors=011

"""
import os
import errno
import numpy as np
import math
import rospy
import tf
from geometry_msgs.msg import PoseStamped


from e_motion_perception_msgs.msg import FloatOccupancyGrid
from hybrid_simulation.msg import VehicleStatus
from hybrid_simulation.msg import VehicleStatusArray
from hybrid_simulation.msg import Observations


class ScenarioParams(object):
    def __init__(self):
        self.ego_vehicle_vel = 0
        self.other_vehicle_vel = 0


class Vehicle(object):

    def __init__(self, vehicle_name):
        self.vehicle_id = vehicle_name
        self.length = 5.0
        self.width = 3.0
        self.box_coord = np.array(
            [[self.length / 2, -self.width / 2],
             [self.length / 2, self.width / 2],
             [-self.length / 2, self.width / 2],
             [-self.length / 2, -self.width / 2]])
        self.vel = 0
        self.lane = 0
        self.pose = PoseStamped()
        self.vehicles_subscriber = rospy.Subscriber('/vehicles_status',
                                                    VehicleStatusArray,
                                                    self.veh_callback)

    def veh_callback(self, vehicles_array):
        # print(vehicles_array)
        obj = [VehicleStatus for VehicleStatus in vehicles_array.VehiclesDetected if VehicleStatus.vehicle_id == self.vehicle_id]
        if not obj:
            return
        vehicle_obj = obj[0]
        # print("vehicle_obj")
        self.pose.pose.position.x = vehicle_obj.pos_x
        self.pose.pose.position.y = vehicle_obj.pos_y
        self.pose.pose.position.z = 0.0
        quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, vehicle_obj.heading)
        self.pose.pose.orientation.x = quaternion[0]
        self.pose.pose.orientation.y = quaternion[1]
        self.pose.pose.orientation.z = quaternion[2]
        self.pose.pose.orientation.w = quaternion[3]
        self.pose.header.frame_id = vehicle_obj.vehicle_id
        # print(self.l, self.w, self.h, self.frame_id)
        self.vel = vehicle_obj.velocity
        self.lane = vehicle_obj.lane
        # print(self.vel)


class Grid(object):

    def __init__(self, x_min, y_min, res, ego_car):
        self.ego_car = ego_car
        self.grid_x_min = x_min
        self.grid_y_min = y_min
        self.resolution = res
        self.tf_listener = tf.TransformListener()
        self.grid_sub = rospy.Subscriber('/prius/state_grid', FloatOccupancyGrid, self.callback)
        self.observations = Observations()
        self.prev_time_stamp = 0.0
        self.skips = 0.0

    def get_status_box(self, obs_center_x, obs_center_y, risk_grid):

        obs_center_x = obs_center_x - self.grid_x_min
        obs_center_y = obs_center_y - self.grid_y_min
        center = np.array([obs_center_x, obs_center_y])
        box_coord = self.ego_car.box_coord + center  # bounding box coordinate of the target vehicle w.r.t grid origin
        box_coord = box_coord / self.resolution  # bounding box coordinate cell number

        x_max, y_max = box_coord.max(axis=0).astype(int)
        x_min, y_min = box_coord.min(axis=0).astype(int)
        # check that bounding box is not out of grid index
        if x_min < 0 or y_min < 0 or x_max > risk_grid.info.height or y_max > risk_grid.info.width:
            print("target is out of grid")
            return

        if risk_grid.header.stamp.to_sec() == self.prev_time_stamp:
            self.skips += 1
            return

        risk_arr = (np.array(risk_grid.data) * 255).astype('uint8')
        risk_arr = risk_arr.reshape(risk_grid.info.height, risk_grid.info.width, risk_grid.nb_channels)

        # risk_prob  = risk_arr[ymin:ymax, xmin:xmax, :3]
        max_risk_1sec = risk_arr[y_min:y_max, x_min:x_max, 0].max()
        # max_risk_2sec = risk_arr[y_min:y_max, x_min:x_max, 1].max()
        # max_risk_3sec = risk_arr[y_min:y_max, x_min:x_max, 2].max()
        return max_risk_1sec

    def callback(self, risk_grid):

        self.observations.front = self.get_status_box(10.0, 0.0, risk_grid)
        self.observations.front_left = self.get_status_box(10.0, -3.0, risk_grid)
        self.observations.front_right = self.get_status_box(10.0, 3.0, risk_grid)
        self.observations.center_left = self.get_status_box(2.5, -3.0, risk_grid)
        self.observations.center_right = self.get_status_box(2.5, 3.0, risk_grid)
        self.observations.back_left = self.get_status_box(-5.0, -3.0, risk_grid)
        self.observations.back_right = self.get_status_box(-5.0, 3.0, risk_grid)
        self.prev_time_stamp = risk_grid.header.stamp.to_sec()
        # print("front ", self.observations.front)
        # print("front_left ", self.observations.front_left)
        # print("front_right ", self.observations.front_right)
        # print("center_left ", self.observations.center_left)
        # print("center_right ", self.observations.center_right)
        # print("back_left ", self.observations.back_left)
        # print("back_right ", self.observations.back_right)
        #
        #

        # test_grid = risk_grid
        # test_arr = np.zeros_like(risk_arr)
        # test_arr[ymin:ymax,xmin:xmax,:] = 1
        # test_arr = test_arr.flatten().tolist()
        # test_grid.data = test_arr
        # pub.publish(test_grid)


def compute_collision_risk(vehicle_a, vehicle_b):
    a_x = vehicle_a.pose.pose.position.x
    a_y = vehicle_a.pose.pose.position.y
    a_vx = vehicle_a.vel_x
    a_vy = vehicle_a.vel_y

    b_x = vehicle_b.pose.pose.position.x
    b_y = vehicle_b.pose.pose.position.y
    b_vx = vehicle_b.vel_x
    b_vy = vehicle_b.vel_y

    # print("zoe:", zoe.vel)
    # print("target", target.vel)

    will_collide = False
    collision_dist = abs(math.sqrt((vehicle_a.length/2 + vehicle_b.width/2)**2 + (vehicle_a.width/2 + vehicle_b.length/2)**2))
    # collision_dist = zoe.l/2 + target.l/2

    # time to reach the minimum distance
    denominator = (a_vx**2 - 2*a_vx*b_vx + b_vx**2 + a_vy**2 - 2*a_vy*b_vy + b_vy**2)
    if denominator == 0:
        min_dist_time = -1
        return will_collide, min_dist_time

    min_dist_time = -(a_x*a_vx - a_vx*b_x - (a_x - b_x)*b_vx + a_y*a_vy - a_vy*b_y - (a_y - b_y)*b_vy)/denominator

    t = min_dist_time
    min_dist = math.sqrt((t*a_vx - t*b_vx + a_x - b_x)**2 + (t*a_vy - t*b_vy + a_y - b_y)**2)
    # print("minidist: ", minDist)
    if min_dist < collision_dist:
        will_collide = True
    return will_collide, min_dist_time


if __name__ == '__main__':

    rospy.init_node('get_cmcdot_observations', anonymous=True)
    rate = rospy.Rate(1)  # 10hz
    ego_vehicle = Vehicle("prius")
    observations_pub = rospy.Publisher('/observations', Observations, queue_size=1)
    my_risk_grid = Grid(-28, -28, 0.1, ego_vehicle)
    goal_pos = 30.0

    while not rospy.is_shutdown():

        current_obs = Observations()
        current_obs.lane = ego_vehicle.lane
        current_obs.dist_goal = ego_vehicle.pose.pose.position.x - goal_pos
        current_obs.front = my_risk_grid.observations.front
        current_obs.front_left = my_risk_grid.observations.front_left
        current_obs.front_right = my_risk_grid.observations.front_right
        current_obs.center_left = my_risk_grid.observations.center_left
        current_obs.center_right = my_risk_grid.observations.center_right
        current_obs.back_left = my_risk_grid.observations.back_left
        current_obs.back_right = my_risk_grid.observations.back_right
        print(current_obs)
        observations_pub.publish(current_obs)
        rate.sleep()
