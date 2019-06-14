#!/usr/bin/env python
# @file    control_other_vehicles.py
# @author  Mario Garzon


from __future__ import absolute_import
from __future__ import print_function

import optparse
import os
import sys
from math import radians


from rospkg import RosPack
import tf
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion

# we need to import python modules from the $SUMO_HOME/tools directory
try:
    sys.path.append(os.path.join(os.path.dirname(
        __file__), '..', '..', '..', '..', "tools"))  # tutorial in tests
    sys.path.append(os.path.join(os.environ.get("SUMO_HOME", os.path.join(
        os.path.dirname(__file__), "..", "..", "..")), "tools"))  # tutorial in docs
    from sumolib import checkBinary
except ImportError:
    sys.exit(
        "please declare environment variable 'SUMO_HOME'"
        " as the root directory of your sumo installation (it should contain folders 'bin', 'tools' and 'docs')")

# Import components from the package.

from hybrid_simulation.route_file import generate_route_file
from hybrid_simulation.route_file import generate_route_file_dmaking
from hybrid_simulation.traci_controls import TraciControls
from hybrid_simulation.ego_vehicle import EgoVehicle
from hybrid_simulation.msg import VehicleStatus
from hybrid_simulation.msg import VehicleStatusArray
from hybrid_simulation.ros_components import RosComponents

import traci.constants as tc
import traci


PI = 3.1415926535897

traci_controller = TraciControls()


def publish_tf_timer_callback(event):

    # rospy.loginfo("Timer called at %s", str(event.current_real))

    global ego_vehicle

    br = tf.TransformBroadcaster()
    vehicles_msg_array = VehicleStatusArray()
    vehicles_msg_array.header.stamp = rospy.Time.now()
    vehicles_msg_array.header.frame_id = "world"

    for running_vehicle, subs in traci.vehicle.getAllSubscriptionResults().items():
        if running_vehicle is not None:
            try:
                result_sub = traci.vehicle.getSubscriptionResults(running_vehicle)
            except traci.TraCIException as e:
                rospy.logerr("Error reading vehicle position from SUMO: %s", e.message)
                result_sub = None

            if result_sub is not None:
                v_position = result_sub.get(tc.VAR_POSITION, None)
                v_angle = result_sub.get(tc.VAR_ANGLE, None)
                v_max_vel = result_sub.get(tc.VAR_MAXSPEED, None)
                v_velocity = result_sub.get(tc.VAR_SPEED, None)
                v_lane_id = result_sub.get(tc.VAR_LANE_INDEX, None)
                v_signals = result_sub.get(tc.VAR_SIGNALS, None)
                # print(result_sub)
                if v_position is None \
                        or v_angle is None \
                        or v_max_vel is None \
                        or v_velocity is None \
                        or v_lane_id is None \
                        or v_signals is None:
                    continue

                v_angle = 360 - v_angle
                # rospy.loginfo("Vehicle %s - Pos_x: %.2f Pos_y: %.2f Angle: %.2f rads: %.2f",
                #               running_vehicle, v_position[0], v_position[0], v_angle, radians(v_angle))
                br.sendTransform((v_position[0], v_position[1], 0),
                                 tf.transformations.quaternion_from_euler(0, 0, radians(v_angle)),
                                 rospy.Time.now(),
                                 running_vehicle,
                                 "world")
                vehicle_msg = VehicleStatus()
                vehicle_msg.vehicle_id = running_vehicle
                vehicle_msg.pos_x = v_position[0]
                vehicle_msg.pos_y = v_position[1]
                vehicle_msg.heading = radians(v_angle)
                vehicle_msg.velocity = v_velocity
                vehicle_msg.max_vel = v_max_vel
                vehicle_msg.lane = v_lane_id
                vehicle_msg.signals = v_signals
                vehicles_msg_array.VehiclesDetected.append(vehicle_msg)
                if ego_vehicle is not None:
                    if running_vehicle == ego_vehicle.ego_vehicle_id:
                        # rospy.loginfo("Control egovehicle")
                        if ros_node_comp.use_gazebo is True:
                            # rospy.loginfo("Use Gazebo True")
                            if ros_node_comp.control_from_gazebo is False:
                                ego_vehicle.set_position_in_gazebo(vehicle_msg)

    ros_node_comp.vehicle_status_pub.publish(vehicles_msg_array)


def run(event):
    """execute the TraCI control loop"""
    # while traci.simulation.getMinExpectedNumber() > 0:

    # print("run callback at", str(event.current_real))

    global ego_vehicle

    if traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()

        if ros_node_comp.control_ego_vehicle is True:
            # rospy.loginfo("Control egovehicle")
            if ego_vehicle is not None:
                # rospy.loginfo("Egovehicle not none")
                if ros_node_comp.use_gazebo is True:
                    if ros_node_comp.control_from_gazebo is True:
                        ego_vehicle.read_position_from_gazebo()

        move_nodes = []
        for veh, subs in traci.vehicle.getAllSubscriptionResults().items():
            move_nodes.append(
                (veh, subs[tc.VAR_ROAD_ID], subs[tc.VAR_LANEPOSITION]))

        departed = traci.simulation.getSubscriptionResults(
        )[tc.VAR_DEPARTED_VEHICLES_IDS]

        if departed is not None:
            # print("step", traci_controller.setting.step, " departed >", departed)
            for v in departed:
                traci.vehicle.subscribe(v, [tc.VAR_POSITION, tc.VAR_ANGLE, tc.VAR_ROAD_ID,
                                            tc.VAR_LANEPOSITION, tc.VAR_MAXSPEED, tc.VAR_SPEED,
                                            tc.VAR_LANE_INDEX, tc.VAR_SIGNALS])
                subs = traci.vehicle.getSubscriptionResults(v)
                move_nodes.append((v, subs[tc.VAR_ROAD_ID], subs[tc.VAR_LANEPOSITION]))
                if ros_node_comp.use_gazebo is True:
                    gazebo_synchro(subs, v)

            if ros_node_comp.use_gazebo is True:
                arrived = traci.simulation.getArrivedIDList()
                if arrived is not None:
                    # print("step", traci_controller.setting.step, " departed >", departed)
                    for item_name in arrived:
                        rospy.loginfo("Deleting model: %s", item_name)
                        if ego_vehicle is not None:
                            if item_name == ego_vehicle.ego_vehicle_id:
                                rospy.loginfo("Not Deleting model: %s", item_name)
                                continue
                        try:
                            ros_node_comp.delete_model(item_name)
                            rospy.loginfo("Deleting model OK!")
                        except rospy.ServiceException as e:
                            rospy.logerr("Delete Service call failed: %s", e.message)

        for vehicle_id, edge, pos in move_nodes:
            traci_controller.check_initial_position(vehicle_id, edge, pos)
            vehicle = traci_controller.vehicle_status[vehicle_id]
            if traci_controller.setting.verbose:
                print("vehicle ID ", vehicle_id, "Pos: ", vehicle, "Target: ", vehicle.target)
            if edge == vehicle.target:
                print("vehicle ID", vehicle_id, "Arrived - ")
                traci_controller.restart(vehicle_id, "D1")

        # if traci.trafficlight.getPhase("911") == 2:
        #     # we are not already switching
        #     n_cars = traci.inductionloop.getLastStepVehicleNumber("det0")
        #     if n_cars > 0:
        #         # there is a vehicle from the north, switch
        #         traci.trafficlight.setPhase("911", 3)
        #     else:
        #         # otherwise try to keep green for EW
        #         traci.trafficlight.setPhase("911", 2)
        traci_controller.setting.step += 1


def gazebo_synchro(subs, vehicle_id):

    global ego_vehicle

    if vehicle_id == ros_node_comp.ego_vehicle_id:
        if ros_node_comp.control_ego_vehicle is True:
            rospy.loginfo("Starting control of %s (ego-vehicle)", vehicle_id)
            ego_vehicle = EgoVehicle(ros_node_comp.ego_vehicle_id)
            ego_vehicle.init_ego_car_control(ros_node_comp.control_from_gazebo)
    else:
        rospack1 = RosPack()
        package_path = rospack1.get_path('hybrid_simulation')
        file_xml = open(package_path + "/sdf/models/car/car_model.sdf")
        xml_string = file_xml.read()
        rospy.loginfo("Spawning model: %s", vehicle_id)
        v_position = subs.get(tc.VAR_POSITION, None)
        v_angle = subs.get(tc.VAR_ANGLE, None)
        v_angle_r = radians(360 - v_angle)
        rospy.loginfo("Position: %.2f - %.2f ", v_position[0], v_position[1])
        rospy.loginfo("Ange: %.2f - %.2f ", v_angle, v_angle_r)
        orientation = tf.transformations.quaternion_from_euler(0,
                                                               0,
                                                               v_angle_r)
        item_pose = Pose(Point(x=v_position[0],
                               y=v_position[1],
                               z=0),
                         Quaternion(orientation[0],
                                    orientation[1],
                                    orientation[2],
                                    orientation[3])
                         )
        try:
            ros_node_comp.spawn_model(vehicle_id, xml_string, "", item_pose, "world")
        except rospy.ServiceException as e:
            rospy.logerr("Error in spawn %s -  %s", vehicle_id, e.message)


def get_options():
    opt_parser = optparse.OptionParser()

    opt_parser.add_option("--nogui", action="store_true",
                          default=False, help="run the commandline version of sumo")
    opt_parser.add_option("-v", "--verbose", action="store_true", dest="verbose",
                          default=False, help="tell me what you are doing")
    optns, args = opt_parser.parse_args()
    return optns


# this is the main entry point of this script
if __name__ == "__main__":

    options = get_options()
    try:
        ros_node_comp = RosComponents()
    except rospy.ROSInterruptException:
        pass

    # this script has been called from the command line. It will start sumo as a
    # server, then connect and run
    if options.nogui:
        sumoBinary = checkBinary("sumo")
    else:
        sumoBinary = checkBinary("sumo-gui")

    traci_controller.setting.verbose = options.verbose

    # first, generate the route file for this simulation

    ros_pack = RosPack()
    if rospy.has_param('~route_file_name'):
        route_file_name = rospy.get_param('~route_file_name')
    else:
        rospy.loginfo("SUMO Interface -- Using default route file")
        route_file_name = "network_traci.rou.xml"
    route_file_path = ros_pack.get_path('hybrid_simulation') + "/sumo_files/" + route_file_name

    #  demand per second from different directions
    # p_we = 40. / 60
    # p_ew = 15. / 60
    # p_ns = 10. / 60
    #

    if rospy.has_param('~n_scenario'):
        n_scenario = rospy.get_param('~n_scenario')
    else:
        rospy.loginfo("SUMO Interface -- Using default scenario number")
        n_scenario = 0

    if n_scenario == 0:
        p_we = 50. / 60
        p_ew = 50. / 60
        p_ns = 0. / 60
        max_steps = 200
        generate_route_file(route_file_path, max_steps, p_we, p_ew, p_ns)
    else:
        generate_route_file_dmaking(route_file_path, n_scenario)

    # this is the normal way of using traci. sumo is started as a
    # # subprocess and then the python script connects and runs
    if rospy.has_param('~sumo_config_file_name'):
        sumo_config_file_name = rospy.get_param('~sumo_config_file_name')
    else:
        rospy.loginfo("SUMO Interface -- Using default sumo config file")
        sumo_config_file_name = "network.sumocfg"

    sumo_config_file_path = ros_pack.get_path('hybrid_simulation') + "/sumo_files/" + sumo_config_file_name

    traci.start([sumoBinary, "-c", sumo_config_file_path,
                 "--collision.action", "none"], label="sim_sumo")
    traci.simulation.subscribe()

    traci_controller.setting.step = 0

    # we start with phase 2 where EW has green
    # traci.trafficlight.setPhase("911", 2)

    global ego_vehicle

    ego_vehicle = None

    rospy.Timer(rospy.Duration.from_sec(0.05), publish_tf_timer_callback)
    rospy.Timer(rospy.Duration.from_sec(0.05), run)
    rospy.loginfo("SUMO Interface -- Starting spinner")

    rospy.spin()

    traci.close()
    sys.stdout.flush()
