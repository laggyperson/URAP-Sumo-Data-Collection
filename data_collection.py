"""
Data collection script that runs simulations and saves the data

Author: Phillip Chen
Date: July 2023
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
import argparse
import random 

# Turning off legacy mode
traci.setLegacyGetLeader(False)

# =========================== Helper Functions ===========================
"""
Finds vehicles within specified range in front of given vehicle ID and returns all IDs and gaps as a list. 
Returns empty lists if none exist
Params:
    str vehID : The id the vehicle to start from
    float dist : The furthest vehicle to get data from relative to the given vehicle
Ret:
    List[List] : list of two lists, first holding the ids, the second holding the gaps in meters
"""
def findLeadingNeighbors(vehID, dist):
    ids = []
    gaps = []

    curr_ID = vehID
    dist_from_veh = 0
    while dist_from_veh <= dist:
        info = traci.vehicle.getLeader(curr_ID)
        if len(info[0]) == 0 or info[0] in ids:
            break
        curr_ID = info[0]
        dist_from_veh += info[1]
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
Returns the ids of vehicles in range in the right lane and the gaps between them
Returns empty lists if none exist
Params:
    str vehID : the vehicle in question
    float dist : distance range to search for vehicles in front and behind the given vehicle
Return:
    List[List] : list of two lists, first holding the ids, the second holding the gaps in meters
"""
def rightLaneInfo(vehID, dist):
    right_veh = traci.vehicle.getRightFollowers(vehID) # Picks a follower

    if len(right_veh) == 0:
        right_veh = traci.vehicle.getRightLeaders(vehID)
        if len(right_veh) == 0:
            return [[], []]
    
    # Finding right vehicle that is in range
    right_veh_ID = right_veh[0][0]
    dist_to_right = right_veh[0][1]
    front_ID = right_veh_ID
    while abs(dist_to_right) > dist:
        info = traci.vehicle.getLeader(front_ID)
        if len(info[0]) == 0 or front_ID == right_veh_ID:
            break
        dist_to_right -= info[1]
        if dist_to_right < 0 and abs(dist_to_right) > dist: # Passed the current vehicle, too far ahead
            return [[], []] # Vehicles are too far away
        front_ID = info[0]

    ids = [front_ID]
    gaps = []

    # Finding all vehicles in front of the returned vehicle
    front = front_ID
    temp_dist = dist_to_right
    while abs(temp_dist) < dist:
        info = traci.vehicle.getLeader(front)
        if len(info[0]) == 0 or abs(temp_dist - info[1]) > dist:
            break
        temp_dist -= info[1]
        front = info[0]
        gaps.append(info[1])
        ids.append(front)

    
    # Finding all vehicles behind this returned vehicle
    back = front_ID
    temp_dist = dist_to_right
    while abs(temp_dist) < dist:
        info = traci.vehicle.getLeader(front_ID)
        if len(info[0]) == 0 or abs(temp_dist + info[1]) > dist:
            break
        temp_dist += info[1]
        back = info[0]
        gaps.append(info[1])
        ids.append(back)

    return [ids, gaps]

"""
Returns the ids of vehicles in range in the left lane and the gaps between them
Returns empty lists if none exist
Params:
    str vehID : the vehicle in question
    float dist : distance range to search for vehicles in front and behind the given vehicle
Return:
    List[List] : list of two lists, first holding the ids, the second holding the gaps in meters
"""
def leftLaneInfo(vehID, dist):
    left_veh = traci.vehicle.getLeftFollowers(vehID) # Picks a follower
    
    if len(left_veh) == 0:
        left_veh = traci.vehicle.getLeftLeaders(vehID)
        if len(left_veh) == 0:
            return [[], []]
    
    # Finding left vehicle that is in range
    left_veh_ID = left_veh[0][0]
    dist_to_left = left_veh[0][1]
    front_ID = left_veh_ID
    while abs(dist_to_left) > dist:
        info = traci.vehicle.getLeader(front_ID)
        if len(info[0]) == 0 or front_ID == left_veh_ID:
            break
        dist_to_left -= info[1]
        if dist_to_left < 0 and abs(dist_to_left) > dist: # Passed the current vehicle, too far ahead
            return [[], []] # Vehicles are too far away
        front_ID = info[0]

    ids = [front_ID]
    gaps = []

    # Finding all vehicles in front of the returned vehicle
    front = front_ID
    temp_dist = dist_to_left
    while abs(temp_dist) < dist:
        info = traci.vehicle.getLeader(front)
        if len(info[0]) == 0 or abs(temp_dist - info[1]) > dist:
            break
        temp_dist -= info[1]
        front = info[0]
        gaps.append(info[1])
        ids.append(front)

    
    # Finding all vehicles behind this returned vehicle
    back = front_ID
    temp_dist = dist_to_left
    while abs(temp_dist) < dist:
        info = traci.vehicle.getLeader(front_ID)
        if len(info[0]) == 0 or abs(temp_dist + info[1]) > dist:
            break
        temp_dist += info[1]
        back = info[0]
        gaps.append(info[1])
        ids.append(back)

    return [ids, gaps]

"""
Computes average of given list. Returns -1 if list is empty
Params:
    List l : the list to compute the average of
Ret:
    float : average of list
"""
def computeAverage(l):
    if len(l) == 0:
        return -1
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

"""
Saves the data to a netCDF file specified in arguments. Data type is an xarray Dataset
Params:
    list data : The array to save
    list veh_ids : Array of vehicle ids
    str path : The filepath to
Return:
    bool : True if successfully saved data, False otherwise
"""
def save_data(data, veh_ids, path, step_length):
    if len(data) == 0:
        return False
    
    np_data = np.array(data)
    
    time = np.array([x * step_length for x in range(len(data))])
    data_labels = ["Speed", "Max Speed", "Acceleration",
                   "Traffic Light Distance", "Traffic Light State", "Traffic Light Time to Switch", "Number of Vehicles to Traffic Light",
                   "Number of Leading Vehicles", "Leading Vehicles Average Gap", "Leading Vehicles Average Speed", "Leading Vehicles Average Acceleration",
                   "Number of Right Lane Vehicles", "Right Lane Average Gap", "Right Lane Average Speed", "Right Lane Average Acceleration",
                   "Number of Left Lane Vehicles", "Left Lane Average Gap", "Left Lane Average Speed", "Left Lane Average Acceleration",
                   "Destination Edge", "Distance to Edge", "Destination Reached"]
    veh_ids = np.array(veh_ids)

    xr_data = xr.DataArray(
        data=np_data,
        dims=["Time", "Veh IDs", "Data Labels"],
        coords={
            "Time": time,
            "Veh IDs": veh_ids,
            "Data Labels": data_labels
        }
    )

    xr_ds = xr.Dataset({"Data": xr_data})
    xr_ds.to_netcdf(path)

    return True
    
# =========================== End Helper Functions ===========================

# Arg parser
argparser = argparse.ArgumentParser(description="Specify simulation specs to run and collect data on the simulation")

argparser.add_argument("sumo_cfg_file", 
                       type=str,
                       metavar="C",
                       help = "The simulation to run")
argparser.add_argument("--num-vehicles", "-n",
                       required=True,
                       type = int,
                       metavar = "N",
                       dest = "num_veh",
                       help="Number of vehicles that will be in the simulation")
argparser.add_argument("--step-length", "-s",
                       type=float,
                       default=0.5,
                       metavar="S",
                       dest="step_length",
                       help="The step length of the simulation. Recommended to be above 0.5 seconds.")
argparser.add_argument("--output-file", "-o",
                       type=str,
                       required=True,
                       metavar="O",
                       dest="output_file",
                       help="The file to store the data in. Must end in \'.nc\'")
argparser.add_argument("--run-time", "-t",
                       type=float,
                       required=True,
                       metavar="T",
                       dest="run_time",
                       help="The time to run the simulation for starting from when the data is collected (seconds)")

args = argparser.parse_args()

# The amount of time to run the simulation for
run_time = args.run_time

# Simulation Step Length in seconds
step_len = args.step_length

# Numbe of vehicles in simulation
num_vehs = args.num_veh

# Save path of output file
path = args.output_file

# Constructing sumo command (terminal)
sumoBinary = "sumo-gui"
sumoCmd = [sumoBinary, 
    "-c", args.sumo_cfg_file,
    "--step-length", str(step_len)
]

# Sending command
traci.start(sumoCmd)

# Get vehicle IDs
vehIDList = traci.vehicle.getIDList()

# Dictionary of edges and corresponding length
tmp_edges = traci.edge.getIDList()
# Filtering out internal edges
edges = [e for e in tmp_edges if ":" not in e]
# Target edges for each vehicle
target_edges = {}

# Array holding all data
data = []

# Keeps track of time
t = 0

# Keeps track of simulation time
sim_time = 0

try:
    while sim_time <= run_time:
        start_time = time.time()

        # Updating subscriptions to vehicles
        new_List = traci.vehicle.getIDList()

        # Start collecting data once number of vehicles reached max
        if len(new_List) == num_vehs:
            data_t = []
            sim_time += step_len

        for vehID in new_List:
            if vehID not in vehIDList:
                traci.vehicle.subscribe(vehID, [
                    tc.VAR_NEXT_TLS,
                    tc.VAR_POSITION3D, tc.VAR_SPEED, tc.VAR_ALLOWED_SPEED, tc.VAR_ACCELERATION,
                    tc.VAR_ROAD_ID, tc.VAR_LANEPOSITION, tc.VAR_EDGES, tc.VAR_ROUTE_INDEX
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
                tl_timeChange = traci.trafficlight.getNextSwitch(tl_id) - t
                tl_numBlock = numberVehtoTL(vehID, tl_dist)

            # Vehicle Data
            veh_speed = v[tc.VAR_SPEED] # in m/s
            veh_max_speed = v[tc.VAR_ALLOWED_SPEED] # Need to take into consideration
            veh_accel = v[tc.VAR_ACCELERATION] # How fast vehicle is accelerating

            # Getting neighbor information 
            # Distance range set to the speed of the vehicle (i.e. 25 m if vehicle moving at 25 m/s)
            leading_info = findLeadingNeighbors(vehID, veh_speed)
            leading_number = len(leading_info[0])
            leading_avg_gap = -1 
            leading_avg_speed = -1
            leading_avg_accel = -1
            if len(leading_info[0]) != 0:
                leading_avg_gap = computeAverage(leading_info[1])
                leading_velAccel = avgSpeedAccel(leading_info[0])
                leading_avg_speed = leading_velAccel[0]
                leading_avg_accel = leading_velAccel[1]

            right_lane_info = rightLaneInfo(vehID, veh_speed)
            right_lane_number = len(right_lane_info[0])
            right_lane_avg_gap = -1
            right_lane_avg_speed = -1
            right_lane_avg_accel = -1
            if len(right_lane_info[0]) != 0:
                right_lane_avg_gap = computeAverage(right_lane_info[1])
                right_lane_velAccel = avgSpeedAccel(right_lane_info[0])
                right_lane_avg_speed = right_lane_velAccel[0]
                right_lane_avg_accel = right_lane_velAccel[1]

            left_lane_info = leftLaneInfo(vehID, veh_speed)
            left_lane_number = len(left_lane_info[0])
            left_lane_avg_gap = -1
            left_lane_avg_speed = -1
            left_lane_avg_accel = -1
            if len(left_lane_info[0]) != 0:
                left_lane_avg_gap = computeAverage(left_lane_info[1])
                left_lane_velAccel = avgSpeedAccel(left_lane_info[0])
                left_lane_avg_speed = left_lane_velAccel[0]
                left_lane_avg_accel = left_lane_velAccel[1]
            
            if vehID == "f_0.o": print(leading_info[0], '\n', right_lane_info[0], '\n', left_lane_info[0], '\n', veh_speed, '\n\n')

            # Getting route information
            curr_route = v[tc.VAR_EDGES]
            curr_edge = v[tc.VAR_ROAD_ID]
            curr_pos = v[tc.VAR_LANEPOSITION]
            route_index = v[tc.VAR_ROUTE_INDEX]
            
            # Data Variables
            reached = 'n'
            dest = curr_route[-1]

            if route_index == len(curr_route) - 1:
                reached = 'y'
                dest = random.choice(edges)
                traci.vehicle.changeTarget(vehID, dest)
                target_edges[vehID] = dest
            
            route_length = traci.vehicle.getDrivingDistance(vehID, dest, 0)
            

            # Loading in data for this vehicle
            if len(new_List) == num_vehs:
                data_v = [
                    veh_speed, veh_max_speed, veh_accel,
                    tl_dist, tl_state, tl_timeChange, tl_numBlock,
                    leading_number, leading_avg_gap, leading_avg_speed, leading_avg_accel,
                    right_lane_number, right_lane_avg_gap, right_lane_avg_speed, right_lane_avg_accel,
                    left_lane_number, left_lane_avg_gap, left_lane_avg_speed, left_lane_avg_accel,
                    dest, route_length, reached
                ]
                data_t.append(data_v)


        # Updating list
        vehIDList = new_List

        # Appending Data to main data
        if len(new_List) == num_vehs:
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
    success = save_data(data, vehIDList, path, step_len)
    if success:
        print("Successfully saved data")
    else:
        print("Encountered an issue with saving data")

    if e == "connection closed by SUMO":
        print("Closing Connection")
    else:
        print("Fatal TraCI Error: {}".format(e))
except KeyboardInterrupt:
    print("Saving Data")
    success = save_data(data, vehIDList, path, step_len)
    if success:
        print("Successfully saved data")
    else:
        print("Encountered an issue with saving data")

    print("Closing Simulation and cleaning up actors")
    
    # Destroy all actors
    for vehID in vehIDList:
        traci.vehicle.remove(vehID)

    traci.close()
finally:
    print("Saving Data")
    success = save_data(data, vehIDList, path, step_len)
    if success:
        print("Successfully saved data")
    else:
        print("Encountered an issue with saving data")

    print("Closing Simulation and cleaning up actors")
    
    # Destroy all actors
    for vehID in vehIDList:
        traci.vehicle.remove(vehID)

    traci.close()