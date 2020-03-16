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

# from e_motion_perception_msgs.msg import FloatOccupancyGrid
from e_motion_perception_msgs.msg import VelocityGrid
# from hybrid_simulation.msg import VehicleStatus
from hybrid_simulation.msg import VehicleStatusArray
from hybrid_simulation.msg import Observations


class ScenarioParams(object):
    def __init__(self):
        self.ego_vehicle_vel = 0
        self.other_vehicle_vel = 0


class Vehicle(object):

    def __init__(self, vehicle_name):
        self.vehicle_id = vehicle_name
        self.length = 1.0
        self.width = 2.0
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
        obj = [vehicle_status for vehicle_status in vehicles_array.VehiclesDetected if vehicle_status.vehicle_id == self.vehicle_id]
        if not obj:
            return
        vehicle_obj = obj[0]
        # print(vehicle_obj)
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
        # print "Ego-lane  " + str(self.lane)


class Grid(object):

    def __init__(self, x_min, y_min, res, ego_car):
        self.ego_car = ego_car
        self.grid_x_min = x_min
        self.grid_y_min = y_min
        self.resolution = res
        self.tf_listener = tf.TransformListener()
        # self.grid_sub = rospy.Subscriber('/prius/state_grid', FloatOccupancyGrid, self.callback)
        self.grid_sub = rospy.Subscriber('/prius/velocity_grid', VelocityGrid, self.callback_velocity)
        self.observations = Observations()
        self.prev_time_stamp = 0.0
        self.skips = 0.0

    def get_status_box(self, obs_center_x, obs_center_y, status_grid):

        obs_center_x = obs_center_x - self.grid_x_min
        obs_center_y = obs_center_y - self.grid_y_min
        center = np.array([obs_center_x, obs_center_y])
        box_coord = self.ego_car.box_coord + center  # bounding box coordinate of the target vehicle w.r.t grid origin
        box_coord = box_coord / self.resolution  # bounding box coordinate cell number

        x_max, y_max = box_coord.max(axis=0).astype(int)
        x_min, y_min = box_coord.min(axis=0).astype(int)
        # check that bounding box is not out of grid index
        if x_min < 0 or y_min < 0 or x_max > status_grid.info.width or y_max > status_grid.info.height:
            print("target is out of grid")
            print('x  ', obs_center_x, self.grid_x_min)
            print('y  ', obs_center_y, self.grid_y_min)
            print (center)

            return

        if status_grid.header.stamp.to_sec() == self.prev_time_stamp:
            self.skips += 1
            return

        status_arr = np.array(status_grid.data).astype('float32')
        status_arr = status_arr.reshape(status_grid.info.height, status_grid.info.width, status_grid.nb_channels)
        # print(type(status_arr))
        # print(status_arr.shape)
        #
        # status_cropped = status_arr[y_min:y_max, x_min:x_max, 0]
        # print (status_cropped)
        # status_cropped = status_arr[y_min:y_max, x_min:x_max, 1]
        # print (status_cropped)
        # status_cropped = status_arr[y_min:y_max, x_min:x_max, 2]
        # print (status_cropped)
        # status_cropped = status_arr[y_min:y_max, x_min:x_max, 3]
        # print (status_cropped)
        # print('x ', x_min, x_max, 'Gx', status_grid.info.height, "y ", y_min, y_max, 'Gy', status_grid.info.width)
        static_obs = status_arr[y_min:y_max, x_min:x_max, 0].max()
        dynamic_obs = status_arr[y_min:y_max, x_min:x_max, 1].max()
        # free_space = status_arr[y_min:y_max, x_min:x_max, 2].max()
        # unknown_space = status_arr[y_min:y_max, x_min:x_max, 3].max()
        # print('Dynamic ', dynamic_obs, "Static ", static_obs, "Free ", free_space, "Unk ", unknown_space)

        observation_value = 0
        if static_obs + dynamic_obs > 0.55:
            observation_value = 1

        return observation_value

    def get_vel_status_box(self, obs_center_x, obs_center_y, velocity_grid):

        obs_center_x = obs_center_x - self.grid_x_min
        obs_center_y = obs_center_y - self.grid_y_min
        center = np.array([obs_center_x, obs_center_y])
        box_coord = self.ego_car.box_coord + center  # bounding box coordinate of the target vehicle w.r.t grid origin
        box_coord = box_coord / self.resolution  # bounding box coordinate cell number

        x_max, y_max = box_coord.max(axis=0).astype(int)
        x_min, y_min = box_coord.min(axis=0).astype(int)
        # check that bounding box is not out of grid index
        if x_min < 0 or y_min < 0 or x_max > velocity_grid.info.width or y_max > velocity_grid.info.height:
            print("target is out of grid")
            print('x  ', obs_center_x, self.grid_x_min)
            print('y  ', obs_center_y, self.grid_y_min)
            print (center)

            return

        if velocity_grid.header.stamp.to_sec() == self.prev_time_stamp:
            self.skips += 1
            return

        status_arr = np.array(velocity_grid.data).astype('float32')
        status_arr = status_arr.reshape(velocity_grid.info.height, velocity_grid.info.width, velocity_grid.nb_channels)
        # print(type(status_arr))
        # print(status_arr.shape)
        #
        # status_cropped = status_arr[y_min:y_max, x_min:x_max, 0]
        # print (status_cropped)
        # status_cropped = status_arr[y_min:y_max, x_min:x_max, 1]
        # print (status_cropped)
        # status_cropped = status_arr[y_min:y_max, x_min:x_max, 2]
        # print (status_cropped)
        # status_cropped = status_arr[y_min:y_max, x_min:x_max, 3]
        # print (status_cropped)
        # print('x ', x_min, x_max, 'Gx', status_grid.info.height, "y ", y_min, y_max, 'Gy', status_grid.info.width)
        forward_vel = status_arr[y_min:y_max, x_min:x_max, 0].max() * self.resolution
        # lateral_vel = status_arr[y_min:y_max, x_min:x_max, 1].max()
        # free_space = status_arr[y_min:y_max, x_min:x_max, 2].max()
        # unknown_space = status_arr[y_min:y_max, x_min:x_max, 3].max()
        # if forward_vel > 0.1:
        #     print('Dynamic ', forward_vel)
        # , "Static ", static_obs, "Free ", free_space, "Unk ", unknown_space)

        observation_value = -1
        if forward_vel > 0.1:
            observation_value = forward_vel

        return observation_value

    def callback(self, status_grid):

        self.observations.front = self.get_status_box(6.5, 0.0, status_grid)
        self.observations.front_left = self.get_status_box(6.5, 3.3, status_grid)
        self.observations.front_right = self.get_status_box(6.5, -3.3, status_grid)
        self.observations.center_left = self.get_status_box(1.5, 3.3, status_grid)
        self.observations.center_right = self.get_status_box(1.5, -3.3, status_grid)
        self.observations.rear_left = self.get_status_box(-3.5, 3.3, status_grid)
        self.observations.rear_right = self.get_status_box(-3.5, -3.3, status_grid)
        self.observations.back_left = self.get_status_box(-3.5, 3.3, status_grid)
        self.observations.back_right = self.get_status_box(-3.5, -3.3, status_grid)
        self.prev_time_stamp = status_grid.header.stamp.to_sec()
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
    def callback_velocity(self, velocity_grid):
        self.observations.front = self.get_vel_status_box(6.5, 0.0, velocity_grid)
        self.observations.front_left = self.get_vel_status_box(6.5, 3.3, velocity_grid)
        self.observations.front_right = self.get_vel_status_box(6.5, -3.3, velocity_grid)
        self.observations.center_left = self.get_vel_status_box(1.5, 3.3, velocity_grid)
        self.observations.center_right = self.get_vel_status_box(1.5, -3.3, velocity_grid)
        self.observations.rear_left = self.get_vel_status_box(-3.5, 3.3, velocity_grid)
        self.observations.rear_right = self.get_vel_status_box(-3.5, -3.3, velocity_grid)
        # self.observations.back_left = self.get_vel_status_box(-3.5, 3.3, velocity_grid)
        # self.observations.back_right = self.get_vel_status_box(-3.5, -3.3, velocity_grid)
        self.prev_time_stamp = velocity_grid.header.stamp.to_sec()
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


def adjust_obs_vel(read_velocity, ego_vehicle_vel):
    # obs_velocity = ceil(car_velocity)
    obs_velocity = -1
    if read_velocity > -1:
        diff_velocity = read_velocity - ego_vehicle_vel
        if diff_velocity >= 0:
            obs_velocity = 1
        else:
            obs_velocity = 0
    return obs_velocity


def get_goals(pos_x):
    # print "Ego vehicle pos: " + str(pos_x)
    if 70 <= pos_x < 270:
        g_pos = 70
        g_lane = 0
    if 30 <= pos_x < 70:
        g_pos = 27
        g_lane = 1
    if -50 <= pos_x < 30:
        g_pos = -47
        g_lane = 1
    if -100 <= pos_x < -50:
        g_pos = -97
        g_lane = 0
    if -150 <= pos_x < -100:
        g_pos = -147
        g_lane = 0
    if -200 <= pos_x < -150:
        g_pos = -197
        g_lane = 0
    if -300 < pos_x < -200:
        g_pos = -270
        g_lane = 0
    # print "goal lane: " + str(g_lane)
    # print "goal pos: " + str(g_pos)
    return g_lane, g_pos


if __name__ == '__main__':

    rospy.init_node('get_cmcdot_observations', anonymous=True)
    rate = rospy.Rate(2.0)  # 10hz
    ego_vehicle_id = rospy.get_param('~ego_vehicle_name', "prius")
    ego_vehicle = Vehicle(ego_vehicle_id)
    observations_pub = rospy.Publisher('/cmcdot_observations', Observations, queue_size=1)
    my_risk_grid = Grid(-14, -10, 0.1, ego_vehicle)

    while not rospy.is_shutdown():

        current_obs = Observations()

        goal_lane, goal_pos = get_goals(ego_vehicle.pose.pose.position.x)
        # diff_goal = self.dis_blocked - current_pos_x
        diff_lane = goal_lane - ego_vehicle.lane

        # Compute lane observation
        if diff_lane == 0:
            current_obs.lane = 0
        elif diff_lane > 0:
            current_obs.lane = 1
        else:
            current_obs.lane = -1
        # print "obs lane: " + str(current_obs.lane)

        diff_goal = ego_vehicle.pose.pose.position.x - goal_pos
        # Compute lane observation
        if diff_goal <= 5:
            current_obs.dist_goal = 0
        elif 5 < diff_goal <= 20:
            current_obs.dist_goal = 1
        elif 20 < diff_goal <= 30:
            current_obs.dist_goal = 2
        elif 30 < diff_goal:
            current_obs.dist_goal = 3
        # print "obs pos: " + str(current_obs.dist_goal)

        current_obs.front = adjust_obs_vel(my_risk_grid.observations.front, ego_vehicle.vel)
        current_obs.front_left = adjust_obs_vel(my_risk_grid.observations.front_left, ego_vehicle.vel)
        current_obs.front_right = adjust_obs_vel(my_risk_grid.observations.front_right, ego_vehicle.vel)
        current_obs.center_left = adjust_obs_vel(my_risk_grid.observations.center_left, ego_vehicle.vel)
        current_obs.center_right = adjust_obs_vel(my_risk_grid.observations.center_right, ego_vehicle.vel)
        current_obs.back_left = adjust_obs_vel(my_risk_grid.observations.back_left, ego_vehicle.vel)
        current_obs.back_right = adjust_obs_vel(my_risk_grid.observations.back_right, ego_vehicle.vel)
        current_obs.rear_left = adjust_obs_vel(my_risk_grid.observations.rear_left, ego_vehicle.vel)
        current_obs.rear_right = adjust_obs_vel(my_risk_grid.observations.rear_right, ego_vehicle.vel)
        # print(current_obs)
        observations_pub.publish(current_obs)
        rate.sleep()
