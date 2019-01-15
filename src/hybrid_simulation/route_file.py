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

    with open(route_file_path, "w") as routes:

        # print("""<routes>

        # <vType accel="4.0" decel="6.0" id="Car_130" length="5.0" minGap="2.5" maxSpeed="36.0" sigma="0.9"
        # lcStrategic="0.0" lcSpeedGain="0.9" lcKeepRight="100.01" />
        # <vType accel="4.0" decel="6.0" id="Car_110" length="5.0" minGap="2.5" maxSpeed="30.5" sigma="0.9"
        # lcStrategic="0.0" lcSpeedGain="0.9" lcKeepRight="100.01" />
        # <vType accel="4.0" decel="6.0" id="Car_90" length="5.0" minGap="2.5" maxSpeed="25.0" sigma="0.9"
        # lcStrategic="0.0" lcSpeedGain="0.9" lcKeepRight="100.01" />
        # <vType accel="4.0" decel="6.0" id="Car_70" length="5.0" minGap="2.5" maxSpeed="19.4" sigma="0.9"
        # lcStrategic="0.0" lcSpeedGain="0.9" lcKeepRight="100.01" />
        # <vType accel="4.0" decel="6.0" id="ego-vehicle" length="4.1" minGap="2.0" maxSpeed="36.0" sigma="0.9"
        # lcStrategic="0.0" lcSpeedGain="0.9" lcKeepRight="100.01"/>
        # <route id="route00" edges="e0-100 e100-200 e200-300 e300-400 e400-500 e500-3000"/>
        # <route id="route01" edges="e100-200 e200-300 e300-400 e400-500 e500-3000"/>
        # <route id="route02" edges="e200-300 e300-400 e400-500 e500-3000"/>
        # <route id="route03" edges="e300-400 e400-500 e500-3000"/>
        # <route id="route04" edges="e400-500 e500-3000"/>
        # <route id="route05" edges="e500-3000"/> """, file=routes)

        print("""<routes>

        <vType accel="4.0" decel="6.0" id="Car_130" length="5.0" minGap="2.5" maxSpeed="36.0" sigma="0.9"
        lcStrategic="0.0" lcSpeedGain="0.9" lcKeepRight="100.01" />
        <vType accel="4.0" decel="6.0" id="Car_110" length="5.0" minGap="2.5" maxSpeed="30.5" sigma="0.9"
        lcStrategic="0.0" lcSpeedGain="0.9" lcKeepRight="100.01" />
        <vType accel="4.0" decel="6.0" id="Car_90" length="5.0" minGap="2.5" maxSpeed="25.0" sigma="0.9"
        lcStrategic="0.0" lcSpeedGain="0.9" lcKeepRight="100.01" />
        <vType accel="4.0" decel="6.0" id="Car_70" length="5.0" minGap="2.5" maxSpeed="19.4" sigma="0.9"
        lcStrategic="0.0" lcSpeedGain="0.9" lcKeepRight="100.01" />
        <vType accel="4.0" decel="6.0" id="ego-vehicle" length="4.1" minGap="2.0" maxSpeed="36.0" sigma="0.9"
        lcStrategic="0.0" lcSpeedGain="0.9" lcKeepRight="100.01"/>
        <route id="route00" edges="e500-3000"/> """, file=routes)        

        if has_param('~ego_vehicle_name'):
            ego_vehicle_id = get_param('~ego_vehicle_name')
        else:
            ego_vehicle_id = "prius"

        if n_scenario == 1:
            print('    <vehicle id="car_1" type="Car_90" route="route00" depart="0" departLane="0" departPos="80" departSpeed="10"/>', file=routes)
            print('    <vehicle id="%s" type="ego-vehicle" route="route00" depart="0" departLane="0" departPos="base" departSpeed="10" color="1,1,1"/>' % ego_vehicle_id, file=routes)

        if n_scenario == 2:
            print('    <vehicle id="car_1" type="Car_70" route="route00" depart="0" departLane="0" departPos="1150" departSpeed="19" />', file=routes)
            print('    <vehicle id="car_2" type="Car_130" route="route00" depart="0" departLane="1" departPos="830" departSpeed="32" />', file=routes)
            print('    <vehicle id="car_3" type="Car_130" route="route00" depart="0" departLane="1" departPos="880" departSpeed="32" />', file=routes)
            print('    <vehicle id="car_4" type="Car_130" route="route00" depart="0" departLane="1" departPos="930" departSpeed="32" />', file=routes)
            print('    <vehicle id="car_5" type="Car_90" route="route00" depart="0" departLane="0" departPos="950" departSpeed="22" />', file=routes)
            print('    <vehicle id="car_6" type="Car_90" route="route00" depart="0" departLane="0" departPos="900" departSpeed="22" />', file=routes)
            print('    <vehicle id="%s" type="ego-vehicle" route="route00" depart="0" departLane="0" departPos="1000" departSpeed="25" color="1,1,1"/>' % ego_vehicle_id, file=routes)

        print("</routes>", file=routes)
