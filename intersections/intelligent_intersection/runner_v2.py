#!/usr/bin/env python

import os
import sys
import random
import traci
from plexe import Plexe, ACC, CACC
from utils import communicate
from math import sqrt 

conflict_matrix = {
    0: [],
    1: [4, 8, 10, 11],
    2: [4, 5, 7, 11],
    3: [],
    4: [1, 2, 7, 11],
    5: [2, 7, 8, 10],
    6: [],
    7: [2, 4, 5, 10],
    8: [1, 5, 10, 11],
    9: [],
    10: [1, 5, 7, 8],
    11: [1, 2, 4, 8]
}

VEHICLE_LENGTH = 4
DISTANCE = 6  # inter-vehicle distance
LANE_NUM = 12
PLATOON_SIZE = 1
SPEED = 25  # m/s
V2I_RANGE = 200
PLATOON_LENGTH = VEHICLE_LENGTH * PLATOON_SIZE + DISTANCE * (PLATOON_SIZE - 1)
ADD_PLATOON_PRO = 1
ADD_PLATOON_STEP = 700
MAX_ACCEL = 2.6
STOP_LINE = 15.0

def add_single_platoon(plexe, topology, step, lane):
    for i in range(PLATOON_SIZE):
        vid = f"v.{step // ADD_PLATOON_STEP}.{lane}.{i}"
        routeID = f"route_{lane}"
        traci.vehicle.add(
            vid, routeID, departPos=str(100 - i * (VEHICLE_LENGTH + DISTANCE)),
            departSpeed=str(5), departLane=str(lane % 3), typeID="vtypeauto"
        )
        plexe.set_path_cacc_parameters(vid, DISTANCE, 2, 1, 0.5)
        plexe.set_cc_desired_speed(vid, SPEED)
        plexe.set_acc_headway_time(vid, 1.5)
        plexe.use_controller_acceleration(vid, False)
        plexe.set_fixed_lane(vid, lane % 3, False)
        traci.vehicle.setSpeedMode(vid, 0)
        if i == 0:
            plexe.set_active_controller(vid, ACC)
            traci.vehicle.setColor(vid, (255, 255, 255, 255))  # red
            topology[vid] = {}
        else:
            plexe.set_active_controller(vid, CACC)
            traci.vehicle.setColor(vid, (200, 200, 0, 255))  # yellow
            topology[vid] = {"front": f"v.{step // ADD_PLATOON_STEP}.{lane}.{i-1}", "leader": f"v.{step // ADD_PLATOON_STEP}.{lane}.0"}

def add_platoons(plexe, topology, step):
    for lane in range(LANE_NUM):
        if random.random() < ADD_PLATOON_PRO:
            add_single_platoon(plexe, topology, step, lane)

def compute_leaving_time(veh):
    distance = 400 + PLATOON_LENGTH + STOP_LINE - traci.vehicle.getDistance(veh)
    speed = traci.vehicle.getSpeed(veh) + 0.00001
    return distance * 1.0 / speed

def main():
    sumo_cmd = ['sumo-gui', '--duration-log.statistics', '--tripinfo-output', 'my_output_file.xml', '-c', 'cfg/twoWay6lanes.sumo.cfg']

    traci.start(sumo_cmd)
    plexe = Plexe()
    traci.addStepListener(plexe)

    step = 0
    topology = {}
    serving_list = []
    serving_list_veh_only = []

    while step < 360000:
        traci.simulationStep()

        if step % ADD_PLATOON_STEP == 0:
            add_platoons(plexe, topology, step)

        deleted_veh = []
        for key, value in list(topology.items()):
            if value == {}:
                odometry = traci.vehicle.getDistance(key)
                if (not key in serving_list_veh_only) and (400 - V2I_RANGE <= odometry < 400 - V2I_RANGE + 100):
                    serving_list.append([key, int(key.split(".")[2]), 0, 1])
                    serving_list_veh_only.append(key)
                if odometry > 800:
                    deleted_veh.append(key)

        for veh in deleted_veh:
            veh_time = veh.split(".")[1]
            veh_route = veh.split(".")[2]
            for i in range(PLATOON_SIZE):
                veh_id = f"v.{veh_time}.{veh_route}.{i}"
                del topology[veh_id]

        serving_list[:] = [element for element in serving_list if traci.vehicle.getDistance(element[0]) < 400 + PLATOON_LENGTH + STOP_LINE]
        serving_list_veh_only = [element[0] for element in serving_list]

        for i in range(len(serving_list)):
            veh_i = serving_list[i][0]
            route_i = int(veh_i.split(".")[2])
            priority = serving_list[i][3]
            if priority == 0:
                serving_list[i][2] = compute_leaving_time(veh_i)
            else:
                max_leaving_time = 0.00001
                for j in range(i):
                    veh_j = serving_list[j][0]
                    route_j = int(veh_j.split(".")[2])
                    leaving_time_j = serving_list[j][2]
                    if (route_j in conflict_matrix[route_i]) and (leaving_time_j > max_leaving_time):
                        max_leaving_time = leaving_time_j

                if max_leaving_time == 0.00001:
                    serving_list[i][3] = 0
                    distance = 400 + PLATOON_LENGTH + STOP_LINE - traci.vehicle.getDistance(veh_i)
                    desired_speed = max(0.0, sqrt(2 * MAX_ACCEL * distance + (traci.vehicle.getSpeed(veh_i))**2))
                    plexe.set_cc_desired_speed(veh_i, desired_speed)
                    serving_list[i][2] = max(0.0, (desired_speed - traci.vehicle.getSpeed(veh_i)) / MAX_ACCEL)
                else:
                    distance_to_stop_line = 400 - STOP_LINE - traci.vehicle.getDistance(veh_i)
                    current_speed = max(0.0, traci.vehicle.getSpeed(veh_i) + 0.00001)
                    decel = 2 * (current_speed * max_leaving_time - distance_to_stop_line) / (max_leaving_time ** 2)
                    desired_speed = max(0.0, current_speed - decel * max_leaving_time)
                    plexe.set_cc_desired_speed(veh_i, desired_speed)
                    serving_list[i][2] = max(0.0, (distance_to_stop_line + PLATOON_LENGTH + 2 * STOP_LINE) / current_speed)

        if step % 10 == 1:
            communicate(plexe, topology)

        step += 1

    traci.close()

if __name__ == "__main__":
    main()
