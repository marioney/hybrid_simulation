#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import print_function
from rospy import has_param, get_param
import random

def generate_route_file_dmaking(route_file_path, n_scenario):
    """
    Generates and stores the route file. (.rou.xml)

    :route_file_path: Absolute path to the file
    :n_scenario: number of the scenario to be executed
    """

    random.seed(42)  # make tests reproducible

        #     <vType accel="4.0" decel="6.0" id="ego-vehicle" length="4.1" minGap="2.0" maxSpeed="36.0" sigma="0.9"
        # lcStrategic="0.0" lcSpeedGain="0.9" lcKeepRight="100.01"/>

    with open(route_file_path, "w") as routes:

        print("""<routes>

        <vType accel="4.0" decel="6.0" id="Car_130" length="5.0" minGap="2.5" maxSpeed="36.0" sigma="0.9"
        lcStrategic="0.0" lcSpeedGain="0.9" lcKeepRight="100.01" />
        <vType accel="4.0" decel="6.0" id="Car_110" length="5.0" minGap="2.5" maxSpeed="30.5" sigma="0.9"
        lcStrategic="0.0" lcSpeedGain="0.9" lcKeepRight="100.01" />
        <vType accel="4.0" decel="6.0" id="Car_90" length="5.0" minGap="2.5" maxSpeed="25.0" sigma="0.9"
        lcStrategic="0.0" lcSpeedGain="0.9" lcKeepRight="100.01" />
        <vType accel="4.0" decel="6.0" id="Car_70" length="5.0" minGap="2.5" maxSpeed="19.4" sigma="0.9"
        lcStrategic="0.0" lcSpeedGain="0.9" lcKeepRight="100.01" />
        <vType accel="4.0" decel="6.0" id="ego-vehicle" length="4.1" minGap="-10.0" maxSpeed="36.0" sigma="0.9"
        lcStrategic="0.0" lcSpeedGain="0.9" lcKeepRight="100.01"/>
        <route id="route00" edges="e0-100 e100-200 e200-300 e300-400 e400-500 e500-3000"/>
        <route id="route01" edges="e100-200 e200-300 e300-400 e400-500 e500-3000"/>
        <route id="route02" edges="e200-300 e300-400 e400-500 e500-3000"/>
        <route id="route03" edges="e300-400 e400-500 e500-3000"/>
        <route id="route04" edges="e400-500 e500-3000"/>
        <route id="route05" edges="e500-3000"/> """, file=routes)

        if has_param('~ego_vehicle_name'):
            ego_vehicle_id = get_param('~ego_vehicle_name')
        else:
            ego_vehicle_id = "prius"

        if n_scenario == 1:
            print('    <vehicle id="car_1" type="Car_90" route="route00" depart="0" departLane="0" departPos="100" departSpeed="max"/>', file=routes)
            print('    <vehicle id="%s" type="ego-vehicle" route="route00" depart="0" departLane="0" departPos="base" departSpeed="max" color="1,1,1"/>' % ego_vehicle_id, file=routes)

        if n_scenario == 2:
            print('    <vehicle id="%s" type="ego-vehicle" route="route03" depart="0" color="1,1,1"/>' % ego_vehicle_id, file=routes)
            print('    <vehicle id="car_1" type="Car_110" route="route01" depart="0" />', file=routes)
            print('    <vehicle id="car_2" type="Car_130" route="route00" depart="0" />', file=routes)
            print('    <vehicle id="car_3" type="Car_90" route="route05" depart="0" />', file=routes)

        print("</routes>", file=routes)
