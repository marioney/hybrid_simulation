#!/usr/bin/env python

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
    from sumolib import checkBinary  # noqa
except ImportError:
    sys.exit(
        "please declare environment variable 'SUMO_HOME'"
        " as the root directory of your sumo installation (it should contain folders 'bin', 'tools' and 'docs')")

# Import components from the package.
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
initial_states_vehicles = [] # list of dictionaries
# each dictionary contains {id:'prius', speed: 35.84, max_speed: 36.0, pos: (20.32924471235345, -4.95), angle: 90.0, road_id: 'e0-100', lane_index: 0 }

def dmaking_timer_callback(event):
    global paused_simulation
    if paused_simulation is False:
        rospy.loginfo("Waiting for decision")
        paused_simulation = True

def publish_tf_timer_callback(event):

    # rospy.loginfo("Timer called at %s", str(event.current_real))

    global paused_simulation
    global ego_vehicle
    global pause_simulation_timer
    global simulation_index

    br = tf.TransformBroadcaster()
    vehicles_msg_array = VehicleStatusArray()
    vehicles_msg_array.header.stamp = rospy.Time.now()
    vehicles_msg_array.header.frame_id = "world"

    simulation_time = traci_controller.setting.step * rospy.get_param('~sumo_time_step')

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
                v_max_vel = result_sub.get(tc.VAR_MAXSPEED, None)
                v_velocity = result_sub.get(tc.VAR_SPEED, None)
                v_lane_id = result_sub.get(tc.VAR_LANE_INDEX, None)
                v_signals = result_sub.get(tc.VAR_SIGNALS, None)

                if v_position is None or v_angle is None or v_max_vel is None or v_velocity is None or v_lane_id is None or v_signals is None:
                    continue

                v_angle = 360 - v_angle
                if running_vehicle == ego_vehicle.ego_vehicle_id:
                    v_angle = v_angle + 90
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
                if running_vehicle == ego_vehicle.ego_vehicle_id:
                    vehicle_msg.heading = radians(v_angle)
                else:
                    vehicle_msg.heading = radians(v_angle + 90)
                vehicle_msg.velocity = v_velocity
                vehicle_msg.max_vel = v_max_vel
                vehicle_msg.lane = v_lane_id
                vehicle_msg.signals = v_signals
                vehicles_msg_array.VehiclesDetected.append(vehicle_msg)

    # Is the simulation waiting for a decision?
    waiting_for_decision = paused_simulation
    vehicles_msg_array.awaiting_decision = waiting_for_decision
    vehicles_msg_array.time_simulation = simulation_time
    vehicles_msg_array.simulation_index = simulation_index
    ros_node_comp.vehicle_status_pub.publish(vehicles_msg_array)

def pause():
	programPause = raw_input("Press the <ENTER> key to continue...")

def initialize_subscription_vehicles():
    # All vehicles depart at time=0
    # This function sets up the subscription to all the vehicles in the scene
    # It also stores the initial list of vehicles and their states in the
    # global varible 'initial_states_vehicles'

    # getMinExpectedNumber: 	The number of vehicles which are in the net
    # plus the ones still waiting to start. This number may be smaller than
    # the actual number of vehicles still to come because of delayed route
    # file parsing
    if traci.simulation.getMinExpectedNumber() > 0:
        # initial simulation step needed to make vehicles depart
        traci.simulationStep()
        traci_controller.setting.step += 1

        # get departed vehicles
        departed = traci.simulation.getSubscriptionResults()[tc.VAR_DEPARTED_VEHICLES_IDS]
        print('departed: ',departed)

        # subscribe to departed vehicles
        if departed is not None:
            # print("step", traci_controller.setting.step, " departed >", departed)
            for v in departed:
                traci.vehicle.subscribe(v, [tc.VAR_POSITION, tc.VAR_ANGLE, tc.VAR_ROAD_ID,
                                            tc.VAR_LANEPOSITION, tc.VAR_MAXSPEED, tc.VAR_SPEED,
                                            tc.VAR_LANE_INDEX, tc.VAR_SIGNALS])
                subs = traci.vehicle.getSubscriptionResults(v)
                # move_nodes.append((v, subs[tc.VAR_ROAD_ID], subs[tc.VAR_LANEPOSITION]))
                if ros_node_comp.use_gazebo is True:
                    init_vehicle_model_and_spawn_gazebo(subs, v)

        # save initial states
        for veh, subs in traci.vehicle.getSubscriptionResults().items():
            this_vehicle_data = {
                'id': veh,
                'speed': traci.vehicle.getSpeed(veh),
                'max_speed': subs[65],
                'lane_position': traci.vehicle.getLanePosition(veh),
                'position': traci.vehicle.getPosition(veh),
                'angle': traci.vehicle.getAngle(veh),
                'edge_id': traci.vehicle.getRoadID(veh),
                'route_id': traci.vehicle.getRouteID(veh),
                'lane_id':  traci.vehicle.getLaneID(veh),
                'lane_idx': traci.vehicle.getLaneIndex(veh)
            }
            initial_states_vehicles.append(this_vehicle_data)

        # another simulation step is executed to stay in line with randomize_state_vehicles,
        # which requires an additional step execution to perform the moveToXY and setRouteID

        traci.simulationStep()
        # print('Initial_states_vehicles: ')
        # for elem in initial_states_vehicles:
        #     print(elem)
        # pause()

def randomize_state_vehicles():
    # This function randomizes the initial state of the vehicles, with variability depending on the randomize flag
    print('Entered randomize state vehicles')

    for vehicle_info in initial_states_vehicles[::-1]:
        print('Initial state for vehicle %s', vehicle_info['id'])
        print(vehicle_info)
        this_vehicle_id = vehicle_info['id']
        traci.vehicle.moveToXY(this_vehicle_id, vehicle_info['edge_id'], vehicle_info['lane_idx'],
        vehicle_info['position'][0], vehicle_info['position'][1], vehicle_info['angle'], 2)
        traci.vehicle.setRouteID(this_vehicle_id, vehicle_info['route_id'])

    traci.simulationStep()

    # Set route: changes the vehicle route to given edges list. The first edge in the list has to be the one that the vehicle is at at the moment.
    # for vehicle_info in initial_states_vehicles[::-1]:
    traci.vehicle.setRouteID(this_vehicle_id, vehicle_info['route_id'])
    traci.simulationStep()

    print('check correct repositioning')
    for veh, subs in traci.vehicle.getSubscriptionResults().items():
        this_vehicle_data = {
            'id': veh,
            'speed': traci.vehicle.getSpeed(veh),
            'max_speed': subs[65],
            'lane_position': traci.vehicle.getLanePosition(veh),
            'position': traci.vehicle.getPosition(veh),
            'angle': traci.vehicle.getAngle(veh),
            'edge_id': traci.vehicle.getRoadID(veh),
            'route_id': traci.vehicle.getRouteID(veh),
            'lane_id':  traci.vehicle.getLaneID(veh),
            'lane_idx': traci.vehicle.getLaneIndex(veh)
        }
        print(this_vehicle_data)
    pause()

def run(event):
    global paused_simulation
    global finished_simulation
    global ego_vehicle
    global pause_simulation_timer

    simulation_duration = rospy.get_param('~simulation_duration')

    if not paused_simulation and not finished_simulation:
        traci.simulationStep()

        traci_controller.setting.step += 1
        simulation_time = traci_controller.setting.step * rospy.get_param('~sumo_time_step')
        # print("simulation_time: ", simulation_time)

        if simulation_time > simulation_duration:
            print('The simulation has finished!')
            finished_simulation = True


    elif not finished_simulation:
        rospy.loginfo("Waiting for decision")
        if ego_vehicle.resume_simulation is True:
            paused_simulation = False
            ego_vehicle.resume_simulation = False
            pause_simulation_timer = rospy.Timer(rospy.Duration.from_sec(rospy.get_param('~dmaking_time_step')),
                                                 dmaking_timer_callback, oneshot=True)


def init_vehicle_model_and_spawn_gazebo(subs, vehicle_id):

    global ego_vehicle

    if vehicle_id == ros_node_comp.ego_vehicle_id:
        rospy.loginfo("Initializing %s (ego-vehicle)", vehicle_id)
        ego_vehicle = EgoVehicle(ros_node_comp.ego_vehicle_id, ros_node_comp.control_ego_vehicle)

    else:
        rospack1 = RosPack()
        package_path = rospack1.get_path('hybrid_simulation')
        file_xml = open(package_path + "/sdf/models/car/car_model.sdf")
        xml_string = file_xml.read()
        rospy.loginfo("Spawning model: %s", vehicle_id)
        # To change lane change mode of a vehicle
        if vehicle_id == "car_X":
            traci.vehicle.setLaneChangeMode(vehicle_id, 0)
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

def simulation_handler():
    # this function takes care of executing the requested number of simulations
    print('Entered simulation handler')
    N_simulations = rospy.get_param('~simulation_repetitions')
    simulation_duration = rospy.get_param('~simulation_duration')

    # subscribe to the vehicles involved in the simulation
    initialize_subscription_vehicles()
    print('Initialization is finished!')

    global simulation_index
    for simulation_index in range(1,N_simulations + 1):
        traci_controller.setting.step = 0

        # set the initial position of the vehicles in the simulation
        if simulation_index > 1:
            randomize_state_vehicles()
            print('vehicles have been repositioned')
        pause()
        # Parameter to control a frequency for pausing the simulation
        dmaking_time_step = rospy.get_param('~dmaking_time_step')
        sumo_time_step = rospy.get_param('~sumo_time_step')

        # global variables
        global paused_simulation # the simulation is paused while waiting for a decision of the planner
        paused_simulation = False

        global finished_simulation # the simulation is finished
        finished_simulation = False

        global pause_simulation_timer # timer that pauses the simulation

        if simulation_index == 1:
            tf_timer = rospy.Timer(rospy.Duration.from_sec(0.05), publish_tf_timer_callback)

        run_timer = rospy.Timer(rospy.Duration.from_sec(sumo_time_step), run)
        # pause_simulation_timer = rospy.Timer(rospy.Duration.from_sec(dmaking_time_step), dmaking_timer_callback, oneshot=True)

        # rospy.loginfo("SUMO Interface -- Starting spinner")
        # rospy.spin()

        while not finished_simulation:
            rospy.sleep(2.0)

        # The simulation is finished: stop the timers
        # tf_timer.shutdown()
        run_timer.shutdown()
        # pause_simulation_timer.shutdown()

        paused_simulation = False
        finished_simulation = False

        print('A simulation is finished')
        pause()

    traci.close()
    sys.stdout.flush()


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
        sumoBinary = "sumo"
    else:
        sumoBinary = "sumo-gui"
    #             sumoBinary = checkBinary('sumo')
    # else:
    #     sumoBinary = checkBinary('sumo-gui')

    traci_controller.setting.verbose = options.verbose

    # first, generate the route file for this simulation
    ros_pack = RosPack()
    if rospy.has_param('~route_file_name'):
        route_file_name = rospy.get_param('~route_file_name')
    else:
        rospy.loginfo("SUMO Interface -- Using default route file")
        route_file_name = "network_traci.rou.xml"
    route_file_path = ros_pack.get_path('hybrid_simulation') + "/sumo_files/" + route_file_name

    if rospy.has_param('~n_scenario'):
        n_scenario = rospy.get_param('~n_scenario')
    else:
        rospy.loginfo("SUMO Interface -- Using default scenario number")
        n_scenario = 1

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
    print('Started')
    traci.simulation.subscribe()

    simulation_handler()
