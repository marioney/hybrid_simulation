#!/usr/bin/env python
# @file    control_other_vehicles.py
# @author  Mario Garzon


from __future__ import absolute_import
from __future__ import print_function

import optparse
import os
import sys
from math import radians

import rospkg
import tf
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from gazebo_msgs.srv import DeleteModel, SpawnModel

# we need to import python modules from the $SUMO_HOME/tools directory
try:
    sys.path.append(os.path.join(os.path.dirname(
        __file__), '..', '..', '..', '..', "tools"))  # tutorial in tests
    sys.path.append(os.path.join(os.environ.get("SUMO_HOME", os.path.join(
        os.path.dirname(__file__), "..", "..", "..")), "tools"))  # tutorial in docs
    from sumolib import checkBinary  # noqa
except ImportError:
    sys.exit(
        "please declare environment variable 'SUMO_HOME'"
        " as the root directory of your sumo installation (it should contain folders 'bin', 'tools' and 'docs')")

from hybrid_simulation.route_file import *
from hybrid_simulation.traci_controls import *
from hybrid_simulation.ego_vehicle import *
import traci.constants as tc


PI = 3.1415926535897

traci_controller = TraciControls()


def ros_initialization():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    global delete_model
    global spawn_model
    global use_gazebo
    global control_ego_vehicle

    rospy.init_node('listener', anonymous=True)
    use_gazebo = rospy.get_param('use_gazebo', True)
    control_ego_vehicle = rospy.get_param('control_ego_vehicle', True)
    if use_gazebo is True:
        rospy.loginfo("Waiting for gazebo services...")
        rospy.wait_for_service("/gazebo/delete_model")
        rospy.wait_for_service("/gazebo/spawn_sdf_model")
        delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        spawn_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)


def publish_tf_timer_callback(event):

    # rospy.loginfo("Timer called at %s", str(event.current_real))

    br = tf.TransformBroadcaster()
    for running_vehicle, subs in traci.vehicle.getSubscriptionResults().items():
        if running_vehicle is not None:
            try:
                result_sub = traci.vehicle.getSubscriptionResults(running_vehicle)
            except traci.TraCIException as e:
                rospy.logerr("Error reading vehicle position from SUMO: %s", e.message)
                result_sub = None

            if result_sub is not None:
                # print(result_sub)
                v_position = result_sub.get(tc.VAR_POSITION, None)
                v_angle = result_sub.get(tc.VAR_ANGLE, None)
                if v_angle is not None and v_position is not None:
                    v_angle = 360 - v_angle
                    # rospy.loginfo("Vehicle %s - Pos_x: %.2f Pos_y: %.2f Angle: %.2f rads: %.2f",
                    #               running_vehicle, v_position[0], v_position[0], v_angle, radians(v_angle))
                    br.sendTransform((v_position[0], v_position[1], 0),
                                     tf.transformations.quaternion_from_euler(0, 0, radians(v_angle)),
                                     rospy.Time.now(),
                                     running_vehicle,
                                     "world")


def run(event):
    """execute the TraCI control loop"""
    # while traci.simulation.getMinExpectedNumber() > 0:

    # print("run callback at", str(event.current_real))

    global ego_vehicle
    global delete_model
    global spawn_model
    global use_gazebo
    global control_ego_vehicle

    if traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()

        if control_ego_vehicle is True:
            if ego_vehicle is not None and use_gazebo is True:
                ego_vehicle.read_position_from_gazebo()

        move_nodes = []
        for veh, subs in traci.vehicle.getSubscriptionResults().items():
            move_nodes.append(
                (veh, subs[tc.VAR_ROAD_ID], subs[tc.VAR_LANEPOSITION]))

        departed = traci.simulation.getSubscriptionResults(
        )[tc.VAR_DEPARTED_VEHICLES_IDS]

        if departed is not None:
            # print("step", traci_controller.setting.step, " departed >", departed)
            for v in departed:
                traci.vehicle.subscribe(v, [tc.VAR_POSITION, tc.VAR_ANGLE, tc.VAR_ROAD_ID, tc.VAR_LANEPOSITION])
                subs = traci.vehicle.getSubscriptionResults(v)
                move_nodes.append((v, subs[tc.VAR_ROAD_ID], subs[tc.VAR_LANEPOSITION]))
                if use_gazebo is True:
                    gazebo_synchro(subs, v)
            if use_gazebo is True:
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
                            delete_model(item_name)
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

        if traci.trafficlight.getPhase("911") == 2:
            # we are not already switching
            n_cars = traci.inductionloop.getLastStepVehicleNumber("det0")
            if n_cars > 0:
                # there is a vehicle from the north, switch
                traci.trafficlight.setPhase("911", 3)
            else:
                # otherwise try to keep green for EW
                traci.trafficlight.setPhase("911", 2)
        traci_controller.setting.step += 1


def gazebo_synchro(subs, vehicle_id):

    global spawn_model
    global ego_vehicle

    if rospy.has_param("/ego_vehicle_name"):
        ego_vehicle_id = rospy.get_param("/ego_vehicle_name")
    else:
        ego_vehicle_id = "prius"

    if vehicle_id == ego_vehicle_id:
        if control_ego_vehicle is True:
            rospy.loginfo("Starting control of %s (ego-vehicle)", vehicle_id)
            ego_vehicle = EgoVehicle()
            ego_vehicle.init_ego_car_control()
    else:
        rospack1 = rospkg.RosPack()
        package_path = rospack1.get_path('sumo_gazebo')
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
            spawn_model(vehicle_id, xml_string, "", item_pose, "world")
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
        ros_initialization()
    except rospy.ROSInterruptException:
        pass

    # this script has been called from the command line. It will start sumo as a
    # server, then connect and run
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    traci_controller.setting.verbose = options.verbose

    # first, generate the route file for this simulation

    #  demand per second from different directions
    # p_we = 40. / 60
    # p_ew = 15. / 60
    # p_ns = 10. / 60
    #
    p_we = 50. / 60
    p_ew = 20. / 60
    p_ns = 10. / 60

    rospack = rospkg.RosPack()
    route_file_path = rospack.get_path('control_ego_vehicle') + "/sumo_files/network_traci.rou.xml"

    # route_file_path = "../sumo_files/network_traci.rou.xml"
    max_steps = 200
    generate_route_file(route_file_path, max_steps, p_we, p_ew, p_ns)

    # this is the normal way of using traci. sumo is started as a
    # # subprocess and then the python script connects and runs
    traci.start([sumoBinary, "-c", rospack.get_path('control_ego_vehicle') + "/sumo_files/network.sumocfg",
                 "--collision.action", "none"], label="sim_sumo")
    traci.simulation.subscribe()

    traci_controller.setting.step = 0

    # we start with phase 2 where EW has green
    traci.trafficlight.setPhase("911", 2)

    global ego_vehicle

    ego_vehicle = None

    rospy.Timer(rospy.Duration.from_sec(0.05), publish_tf_timer_callback)
    rospy.Timer(rospy.Duration.from_sec(0.05), run)
    rospy.spin()

    traci.close()
    sys.stdout.flush()
