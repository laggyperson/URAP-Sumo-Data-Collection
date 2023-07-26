"""
Data collection for:
G16
100 vehicles
No lane changing

Phillip Chen: July 2023
"""

import os
import sys
if 'SUMO_HOME' in os.environ:
    sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))
import traci
import traci.constants as tc
import time
import numpy as np
import xarray as xr

# Turning off legacy mode
traci.setLegacyGetLeader(False)

# =========================== Helper Functions ===========================
"""
Finds specified number of vehicles in front of given vehicle ID and returns all IDs and distances as a list.
Returns empty lists if none exist
Params:
    str vehID : The id the vehicle to start from
    int num : number of vehicles to collect data on
Ret:
    List[List] : list of two lists, first holding the ids, the second holding the gaps in meters
"""
def findLeadingNeighbors(vehID, num):
    ids = []
    gaps = []

    curr_ID = vehID
    for _ in range(num):
        info = traci.vehicle.getLeader(curr_ID)
        if len(info[0]) == 0 or info[0] in ids:
            break
        curr_ID = info[0]
        ids.append(info[0])
        gaps.append(info[1])

    return [ids, gaps]

"""
Returns average speed and acceleration of all given vehicles
Params:
    List[str] ids : the ids of vehicles to compute for
Return:
    List[float] : length 2 list holding average speed and average acceleration of vehicles
"""
def avgSpeedAccel(ids):
    speeds, accels = [], []
    
    for vehID in ids:
        speeds.append(traci.vehicle.getSpeed(vehID))
        accels.append(traci.vehicle.getAcceleration(vehID))
    
    return [computeAverage(speeds), computeAverage(accels)]

"""
Returns the ids of specified number of vehicles in the right lane and the gaps between them
Returns empty lists if none exist
Params:
    str vehID : the vehicle in question
    int num : number of vehicles in front and behind on the right lane
Return:
    List[List] : list of two lists, first holding the ids, the second holding the gaps in meters
"""
def rightLaneInfo(vehID, num):
    right_veh = traci.vehicle.getRightFollowers(vehID) # Picks a follower

    if len(right_veh) == 0:
        right_veh = traci.vehicle.getRightLeaders(vehID)
        if len(right_veh) == 0:
            return [[], []]
    
    right_veh = right_veh[0][0]
    res = findLeadingNeighbors(right_veh, num) # Get all vehicles in front of returned vehicle

    # Finding all vehicles behind this returned vehicle
    curr_ID = right_veh
    for _ in range(num):
        info = traci.vehicle.getFollower(curr_ID)
        if len(info[0]) == 0 or info[0] in res[0]:
            break
        curr_ID = info[0]
        res[0].append(info[0])
        res[1].append(info[1])

    return res

"""
Returns the ids of specified number of vehicles in the left lane and the gaps between them. 
Returns empty lists if none exist
Params:
    str vehID : the vehicle in question
Return:
    List[List] : list of two lists, first holding the ids, the second holding the gaps in meters
"""
def leftLaneInfo(vehID, num):
    left_veh = traci.vehicle.getLeftFollowers(vehID) # Picks a follower
    
    if len(left_veh) == 0:
        left_veh = traci.vehicle.getLeftLeaders(vehID)
        if len(left_veh) == 0:
            return [[], []]
    left_veh = left_veh[0][0]
    res = findLeadingNeighbors(left_veh, num) # Get all vehicles in front of returned vehicle

    # Finding all vehicles behind this returned vehicle
    curr_ID = left_veh
    for _ in range(num):
        info = traci.vehicle.getFollower(curr_ID)
        if len(info[0]) == 0 or info[0] in res[0]:
            break
        curr_ID = info[0]
        res[0].append(info[0])
        res[1].append(info[1])

    return res

"""
Computes average of given list
Params:
    List l : the list to compute the average of
Ret:
    float : average of list
"""
def computeAverage(l):
    return sum(l) / len(l)

"""
Finds the number of vehicles between given vehicle and traffic light
Params:
    str vehID : the vehicle observed
    str tl_dist : the distance to the traffic light
Ret:
    int count : number of vehicles between observed vehicle and traffic light
"""
def numberVehtoTL(vehID, tl_dist):
    count = 0

    curr_ID = vehID
    running_dist = 0
    while True:
        info = traci.vehicle.getLeader(curr_ID)
        curr_ID = info[0]
        running_dist += info[1]
        if len(curr_ID) == 0 or running_dist > tl_dist:
            break
        count += 1

    return count

# =========================== End Helper Functions ===========================

# Arg parser
arg_parser = argparse.ArgumentParser(description="Specify simulation specs to run and collect data on the simulation")

# Simulation Step Length in seconds
step_len = 0.5

# Numbe of vehicles in simulation
num_vehs = 100

# Constructing sumo command (terminal)
sumoBinary = "sumo-gui"
sumoCmd = [sumoBinary, 
    "-c", "sumocfg/G16_100v_noLC.sumocfg",
    "--step-length", str(step_len)
]

# Sending command
traci.start(sumoCmd)

# Get vehicle IDs
vehIDList = traci.vehicle.getIDList()

# Array holding all data
data = []

# Keeps track of time
t = 0

try:
    while True:
        start_time = time.time()

        # Updating subscriptions to vehicles
        new_List = traci.vehicle.getIDList()

        # Start collecting data once number of vehicles reached max
        if len(new_List) == num_vehs:
            data_t = []

        # # Neighbors whose data already calculated list for efficiency
        # calc_neighbors = []

        # Induction Loop detector: see which vehicles have made the loop
        passed_veh = traci.inductionloop.getLastStepVehicleIDs("e1_1") + traci.inductionloop.getLastStepVehicleIDs("e1_2")

        for vehID in new_List:
            if vehID not in vehIDList:
                traci.vehicle.subscribe(vehID, [
                    tc.VAR_NEXT_TLS,
                    tc.VAR_POSITION3D, tc.VAR_SPEED, tc.VAR_ALLOWED_SPEED, tc.VAR_ACCELERATION,
                ])
        
            v = traci.vehicle.getSubscriptionResults(vehID)
            
            # Traffic light Data
            # Default Values
            tl_id, tl_dist, tl_state, tl_timeChange, tl_numBlock = -1, -1, 'n', -1, 0
            if len(v[tc.VAR_NEXT_TLS]) > 0:
                tl = v[tc.VAR_NEXT_TLS][0] # Only 1 traffic light per intersection at Gomentum
                tl_id = tl[0] # Id of next TL
                tl_dist = tl[2] # Distance in meters
                tl_state = tl[3] # string 'r', 'g', or 'y' (for more, see documentation)
                # Discretize traffic light states
                if tl_state == 'r':
                    tl_state = 1
                elif tl_state == 'y':
                    tl_state = 2
                elif tl_state == 'g':
                    tl_state = 3
                tl_timeChange = traci.trafficlight.getNextSwitch(tl_id) - t
                tl_numBlock = numberVehtoTL(vehID, tl_dist)

            # Vehicle Data
            veh_speed = v[tc.VAR_SPEED] # in m/s
            veh_max_speed = v[tc.VAR_ALLOWED_SPEED] # Need to take into consideration
            veh_accel = v[tc.VAR_ACCELERATION] # How fast vehicle is accelerating

            # Getting neighbor information 
            leading_info = findLeadingNeighbors(vehID, 5)
            leading_avg_gap = -1 
            leading_avg_speed = -1
            leading_avg_accel = -1
            if len(leading_info[0]) != 0:
                leading_avg_gap = computeAverage(leading_info[1])
                leading_velAccel = avgSpeedAccel(leading_info[0])
                leading_avg_speed = leading_velAccel[0]
                leading_avg_accel = leading_velAccel[1]

            right_lane_info = rightLaneInfo(vehID, 5)
            right_lane_avg_gap = -1
            right_lane_avg_speed = -1
            right_lane_avg_accel = -1
            if len(right_lane_info[0]) != 0:
                right_lane_avg_gap = computeAverage(right_lane_info[1])
                right_lane_velAccel = avgSpeedAccel(right_lane_info[0])
                right_lane_avg_speed = right_lane_velAccel[0]
                right_lane_avg_accel = right_lane_velAccel[1]

            left_lane_info = leftLaneInfo(vehID, 5)
            left_lane_avg_gap = -1
            left_lane_avg_speed = -1
            left_lane_avg_accel = -1
            if len(left_lane_info[0]) != 0:
                left_lane_avg_gap = computeAverage(left_lane_info[1])
                left_lane_velAccel = avgSpeedAccel(left_lane_info[0])
                left_lane_avg_speed = left_lane_velAccel[0]
                left_lane_avg_accel = left_lane_velAccel[1]

            # Checking if vehicle passed induction loop detector (unfortunately hard coded)
            passsed = 0
            if vehID in passed_veh:
                passed = 1

            # Loading in data for this vehicle
            if len(new_List) == num_vehs:
                data_v = [
                    veh_speed, veh_max_speed, veh_accel,
                    tl_dist, tl_state, tl_timeChange, tl_numBlock,
                    leading_avg_gap, leading_avg_speed, leading_avg_accel,
                    right_lane_avg_gap, right_lane_avg_speed, right_lane_avg_accel,
                    left_lane_avg_gap, left_lane_avg_speed, left_lane_avg_accel,
                    passed
                ]
                data_t.append(data_v)


        # Updating list
        vehIDList = new_List

        # Appending Data to main data
        if len(new_list) == num_vehs:
            data.append(data_t)

        # Time control
        end_time = time.time()
        elapsed = end_time - start_time
        if elapsed < step_len:
            time.sleep(step_len - elapsed)
        
        traci.simulationStep() # Tick the simulation every 0.05 seconds
        t += step_len # Assuming that this operation takes negligible time

except traci.exceptions.FatalTraCIError as e:
    print("Saving Data")
    save_data()
    if e == "connection closed by SUMO":
        print("Closing Connection")
    else:
        print("Fatal TraCI Error: {}".format(e))
except KeyboardInterrupt:
    print("Saving Data")
    save_data()
    print("Closing Simulation and cleaning up actors")
    
    # Destroy all actors
    for vehID in vehIDList:
        traci.vehicle.remove(vehID)

    traci.close()
