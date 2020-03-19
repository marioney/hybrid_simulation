#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import print_function
from rospy import has_param, get_param
from random import uniform, seed


def generate_route_file(route_file_path, max_steps, p_we, p_ew, p_ns):
    """
    Generates and stores the route file. (.rou.xml)

    :route_file_path: Absolute path to the file
    :max_steps: Maximum number of steps
    :p_we: Probability of having a car in the we init point
    :p_ew: Probability of having a car in the ew init point
    :p_ns: Probability of having a car in the ns init point
    """

    # random.seed(42)  # make tests reproducible
    seed()  # make tests reproducible

    with open(route_file_path, "w") as routes:
        #
        # print("""<routes>
        #
        #         <vType accel="2.0" decel="6.0" id="Car1" length="5.0" minGap="3.5" maxSpeed="5.0" sigma="1.0"
        #         lcStrategic="0.0" lcCooperative="0.45" lcSpeedGain="0.009" lcKeepRight="0.001" />
        #         <vType accel="2.0" decel="6.0" id="ego-vehicle" length="5.0" minGap="3.5" maxSpeed="5.0" sigma="0.0"
        #         lcStrategic="0.0" lcCooperative="0.5" lcSpeedGain="0.009" lcKeepRight="0.001"/>
        #         <route id="route01" edges="EM1 EM2 EM3 EM4 EM5 EM6"/>
        #         <route id="route02" edges="EM4 EM5 EM6"/>
        #         <route id="routeEgo" edges="EI0 EI1 EM1 EM2 EM3 EM4 EM5 EO0 EO1"/>
        #         <route id="route03" edges="EM5 EO0 EO1"/> """, file=routes)

        print("""<routes>

        <vType accel="2.0" decel="6.0" id="Car1" length="5.0" minGap="3.5" maxSpeed="5.0" sigma="0.9"
        lcStrategic="0.0" lcSpeedGain="0.009" lcKeepRight="0.001" />
        <vType accel="2.0" decel="6.0" id="ego-vehicle" length="5.0" minGap="3.5" maxSpeed="5.0" sigma="0.9"
        lcStrategic="0.0" lcSpeedGain="0.009" lcKeepRight="0.001"/>
        <route id="route01" edges="EM1 EM2 EM3 EM4 EM5 EM6"/>
        <route id="route02" edges="EM4 EM5 EM6"/>
        <route id="routeEgo" edges="EI0 EI1 EM1 EM2 EM3 EM4 EM5 EO0 EO1"/>
        <route id="route03" edges="EM5 EO0 EO1"/> """, file=routes)

        veh_nr = 0
        for i in range(2, max_steps):
            # r_val = uniform(0.1, 1.1)
            # print("Ran val " + str(r_val))
            if uniform(0.1, 1.1) < (p_we + 0.1):
                print('    <vehicle id="right_%i" type="Car1" route="route01" depart="%i" departLane="1" />'
                      % (veh_nr, i), file=routes)
                veh_nr += 1
            if uniform(0.1, 1.1) < (p_ew + 0.1):
                print('    <vehicle id="left_%i" type="Car1" route="route02" depart="%i" departLane="0" />'
                      % (veh_nr, i), file=routes)
                veh_nr += 1
            if uniform(0.1, 1.1) < (p_ew + 0.1):
                print('    <vehicle id="left_%i" type="Car1" route="route03" depart="%i" departLane="0" />'
                      % (veh_nr, i), file=routes)
                veh_nr += 1
            # if random.uniform(0, 1) < p_ns:
            #     print('    <vehicle id="down_%i" type="Car1" route="route03" depart="%i" />'
            #           % (veh_nr, i), file=routes)
            #     veh_nr += 1

            # Add broken vehicle after 30? steps
            #
            # if i in [10, 17, 24, 31]:
            #     print('    <vehicle id="broken_%i" type="Car1" route="route0%i" depart="%i"/>'
            #           % (veh_nr, (i % 2)+1, i), file=routes)
            #     veh_nr += 1

            # if i == 45:
            if has_param("/ego_vehicle_name"):
                ego_vehicle_id = get_param("/ego_vehicle_name")
            else:
                ego_vehicle_id = "prius"

            if i == 10:
                print('    <vehicle id="%s" type="ego-vehicle" route="routeEgo" depart="%i" color="1,1,1"/>'
                      % (ego_vehicle_id, i), file=routes)
                veh_nr += 1

        print("</routes>", file=routes)


def generate_route_file_dmaking(route_file_path, n_scenario):
    """uniform(0.1, 1.1)
    Generates and stores the route file. (.rou.xml)

    :route_file_path: Absolute path to the file
    :max_steps: Maximum number of steps
    :p_we: Probability of having a car in the we init point
    :p_ew: Probability of having a car in the ew init point
    :p_ns: Probability of having a car in the ns init point
    """

    # random.seed(42)  # make tests reproducible

    with open(route_file_path, "w") as routes:

        print("""<routes>

        <vType accel="2.0" decel="6.0" id="Car_130" length="5.0" minGap="2.5" maxSpeed="13.0" sigma="0.9"
        lcStrategic="0.0" lcSpeedGain="0.9" lcKeepRight="100.01" />
        <vType accel="2.0" decel="6.0" id="Car_100" length="5.0" minGap="2.5" maxSpeed="10.0" sigma="0.9"
        lcStrategic="0.0" lcSpeedGain="0.9" lcKeepRight="100.01" />
        <vType accel="2.0" decel="6.0" id="Car_80" length="5.0" minGap="2.5" maxSpeed="8.0" sigma="0.9"
        lcStrategic="0.0" lcSpeedGain="0.9" lcKeepRight="100.01" />
        <vType accel="2.0" decel="6.0" id="Car_70" length="5.0" minGap="2.5" maxSpeed="7.0" sigma="0.9"
        lcStrategic="0.0" lcSpeedGain="0.9" lcKeepRight="0.01" />
        <vType accel="1.0" decel="6.0" id="Car_50" length="5.0" minGap="2.5" maxSpeed="5.0" sigma="0.9"
        lcStrategic="0.0" lcSpeedGain="0.9" lcKeepRight="0.01" />
        <vType accel="1.0" decel="6.0" id="ego-vehicle" length="5.0" minGap="2.5" maxSpeed="13.0" sigma="0.9"
        lcStrategic="0.0" lcSpeedGain="0.9" lcKeepRight="100.01"/>
        <route id="route01" edges="D8 L8 L9 L11 L1 D1"/>
        <route id="route02" edges="L8 L9 L11 L1 D1"/>
        <route id="route03" edges="L9 L11 L1 D1"/>
        <route id="route04" edges="L11 L1 D1"/>
        <route id="route05" edges="L1 D1"/> """, file=routes)

        if has_param('~ego_vehicle_name'):
            ego_vehicle_id = get_param('~ego_vehicle_name')
        else:
            ego_vehicle_id = "prius"

        if n_scenario == 1:
            print('    <vehicle id="%s" type="ego-vehicle" route="route01" depart="1"'
                  '  departSpeed="max" color="1,1,1" departLane="0"/>' % ego_vehicle_id, file=routes)
            print('    <vehicle id="car_80" type="Car_80" route="route03" depart="1" />', file=routes)

        if n_scenario == 2:
            print('    <vehicle id="%s" type="ego-vehicle" route="route01" depart="1"'
                  '  departSpeed="max" color="1,1,1" departLane="0"/>' % ego_vehicle_id, file=routes)
            print('    <vehicle id="car_70" type="Car_70" route="route05" depart="1" />', file=routes)
            print('    <vehicle id="car_100" type="Car_100" route="route03" depart="1" />', file=routes)

        if n_scenario == 3:
            print('    <vehicle id="%s" type="ego-vehicle" route="route01" depart="1"'
                  '  departSpeed="max" color="1,1,1" departLane="0"/>' % ego_vehicle_id, file=routes)
            print('    <vehicle id="car_70" type="Car_80" route="route05" depart="1" />', file=routes)
            print('    <vehicle id="car_100" type="Car_130" route="route02" depart="1" />', file=routes)
        if n_scenario == 4:
            print('    <vehicle id="%s" type="ego-vehicle" route="route01" depart="1"'
                  '  departSpeed="max" color="1,1,1" departLane="0"/>' % ego_vehicle_id, file=routes)
            print('    <vehicle id="car_1" type="Car_70" route="route02" depart="3"  departLane="1"/>', file=routes)
            print('    <vehicle id="car_2" type="Car_80" route="route03" depart="3" departLane="1"/>', file=routes)
            print('    <vehicle id="car_3" type="Car_100" route="route04" depart="3" departLane="1"/>', file=routes)
            print('    <vehicle id="car_4" type="Car_130" route="route05" depart="3" departLane="1"/>', file=routes)
            print('    <vehicle id="car_5" type="Car_130" route="route03" depart="3" departLane="1"/>', file=routes)
            print('    <vehicle id="car_6" type="Car_130" route="route03" depart="3" departLane="1"/>', file=routes)
            print('    <vehicle id="car_7" type="Car_70" route="route04" depart="4" departLane="1"/>', file=routes)
            print('    <vehicle id="car_8" type="Car_80" route="route05" depart="4" departLane="1"/>', file=routes)
            print('    <vehicle id="car_9" type="Car_100" route="route02" depart="4" departLane="1"/>', file=routes)
            print('    <vehicle id="car_10" type="Car_130" route="route03" depart="4" departLane="1"/>', file=routes)
            print('    <vehicle id="car_11" type="Car_130" route="route04" depart="4" departLane="1"/>', file=routes)
            print('    <vehicle id="car_12" type="Car_130" route="route05" depart="4" departLane="1"/>', file=routes)
            print('    <vehicle id="car_13" type="Car_70" route="route02" depart="5" departLane="1"/>', file=routes)
            print('    <vehicle id="car_14" type="Car_80" route="route03" depart="5" departLane="1"/>', file=routes)
            print('    <vehicle id="car_15" type="Car_100" route="route04" depart="5" departLane="1"/>', file=routes)
            print('    <vehicle id="car_16" type="Car_130" route="route05" depart="5" departLane="1"/>', file=routes)
            print('    <vehicle id="car_17" type="Car_70" route="route02" depart="6" departLane="1"/>', file=routes)
            print('    <vehicle id="car_18" type="Car_80" route="route03" depart="6" />', file=routes)
            print('    <vehicle id="car_19" type="Car_100" route="route04" depart="6" />', file=routes)
            print('    <vehicle id="car_20" type="Car_130" route="route05" depart="6" />', file=routes)
            print('    <vehicle id="car_21" type="Car_130" route="route03" depart="6" />', file=routes)
            print('    <vehicle id="car_22" type="Car_130" route="route03" depart="6" />', file=routes)
            print('    <vehicle id="car_23" type="Car_70" route="route04" depart="7" />', file=routes)
            print('    <vehicle id="car_24" type="Car_80" route="route05" depart="7" />', file=routes)
            print('    <vehicle id="car_25" type="Car_100" route="route02" depart="7" />', file=routes)
            print('    <vehicle id="car_26" type="Car_130" route="route03" depart="7" />', file=routes)
            print('    <vehicle id="car_27" type="Car_130" route="route04" depart="7" />', file=routes)
            print('    <vehicle id="car_28" type="Car_130" route="route05" depart="7" />', file=routes)
            print('    <vehicle id="car_29" type="Car_70" route="route02" depart="8" />', file=routes)
            print('    <vehicle id="car_30" type="Car_80" route="route03" depart="8" />', file=routes)
            print('    <vehicle id="car_31" type="Car_100" route="route04" depart="8" />', file=routes)
            print('    <vehicle id="car_32" type="Car_130" route="route05" depart="8" />', file=routes)
        if n_scenario == 5:
            print('    <vehicle id="%s" type="ego-vehicle" route="route01" depart="1"'
                  '  departSpeed="max" color="1,1,1" departLane="0"/>' % ego_vehicle_id, file=routes)
            print('    <vehicle id="car_1" type="Car_70" route="route05" depart="3"'
                  ' departLane="0" departSpeed="max" />', file=routes)
            print('    <vehicle id="car_2" type="Car_130" route="route05" depart="3"'
                  ' departLane="0" departSpeed="max" />', file=routes)
            print('    <vehicle id="car_3" type="Car_130" route="route04" depart="3"'
                  ' departLane="0" departSpeed="max" />', file=routes)
            print('    <vehicle id="car_4" type="Car_130" route="route02" depart="7"'
                  ' departLane="1" departSpeed="max" />', file=routes)
            print('    <vehicle id="car_5" type="Car_130" route="route02" depart="7"'
                  ' departLane="1" departSpeed="max" />', file=routes)
        if n_scenario == 6:
            print('    <vehicle id="car_6" type="Car_50" route="route04" depart="1"'
                  ' departLane="0" departSpeed="max" />', file=routes)
            print('    <vehicle id="car_7" type="Car_70" route="route02" depart="1"'
                  ' departLane="1" departSpeed="max" />', file=routes)
            print('    <vehicle id="car_8" type="Car_70" route="route02" depart="2"'
                  ' departLane="1" departSpeed="max" />', file=routes)
            print('    <vehicle id="car_9" type="Car_70" route="route02" depart="3"'
                  ' departLane="1" departSpeed="max" />', file=routes)
            print('    <vehicle id="car_10" type="Car_70" route="route02" depart="4"'
                  ' departLane="1" departSpeed="max" />', file=routes)
            print('    <vehicle id="car_11" type="Car_70" route="route02" depart="5"'
                  ' departLane="1" departSpeed="max" />', file=routes)
            print('    <vehicle id="car_12" type="Car_70" route="route02" depart="6"'
                  ' departLane="1" departSpeed="max" />', file=routes)
            print('    <vehicle id="car_13" type="Car_70" route="route02" depart="7"'
                  ' departLane="1" departSpeed="max" />', file=routes)
            print('    <vehicle id="car_14" type="Car_70" route="route02" depart="8"'
                  ' departLane="1" departSpeed="max" />', file=routes)
            print('    <vehicle id="car_15" type="Car_70" route="route02" depart="9"'
                  ' departLane="1" departSpeed="max" />', file=routes)
            print('    <vehicle id="%s" type="ego-vehicle" route="route02" depart="10"'
                  '  departSpeed="max" color="1,1,1" departLane="0"/>' % ego_vehicle_id, file=routes)
            print('    <vehicle id="car_16" type="Car_70" route="route02" depart="10"'
                  ' departLane="1" departSpeed="max" />', file=routes)
            print('    <vehicle id="car_17" type="Car_70" route="route02" depart="11"'
                  ' departLane="1" departSpeed="max" />', file=routes)
            print('    <vehicle id="car_18" type="Car_70" route="route02" depart="12"'
                  ' departLane="1" departSpeed="max" />', file=routes)
            print('    <vehicle id="car_19" type="Car_70" route="route02" depart="13"'
                  ' departLane="1" departSpeed="max" />', file=routes)
            print('    <vehicle id="car_20" type="Car_70" route="route02" depart="14"'
                  ' departLane="1" departSpeed="max" />', file=routes)
            print('    <vehicle id="car_21" type="Car_70" route="route02" depart="15"'
                  ' departLane="1" departSpeed="max" />', file=routes)
            print('    <vehicle id="car_22" type="Car_70" route="route02" depart="16"'
                  ' departLane="1" departSpeed="max" />', file=routes)
            print('    <vehicle id="car_23" type="Car_70" route="route02" depart="17"'
                  ' departLane="1" departSpeed="max" />', file=routes)

        print("</routes>", file=routes)
