from sumolib import checkBinary
import subprocess
import time
import traci
import numpy as np

sumocfg_file = "cnr/exp.sumocfg"
app ="sumo-gui"


command = [checkBinary(app), '-c', sumocfg_file]
command += ['--seed', "1"]
command += ['--no-step-log', 'True']
command += ['--time-to-teleport', '600'] # long teleport for safety
command += ['--no-warnings', 'True']
command += ['--duration-log.disable', 'True']
command += ['--start']
# collect trip info if necessary
# if self.is_record:
#     command += ['--tripinfo-output',
#                 self.output_path + ('%s_%s_trip.xml' % (self.name, self.agent))]


traci.start(command)

# traci.start(command)
step = 0

junction_ids = ["nt" + str(i) for i in range(1, 26)]
lane_ids = {}
for jID in junction_ids:
    lane_ids[jID] = list(set(traci.trafficlight.getControlledLanes(jID)))

while step < 4000:
    step +=1 
    traci.simulationStep()

    print(f"Timestep {step}")




    if step % 5 == 0 :
        emergencies = {}
        for jID in junction_ids : 
            vehicle_dict = {lane_id: traci.lane.getLastStepVehicleIDs(lane_id) for lane_id in lane_ids[jID]}
            for lane_id, vehicle_ids in vehicle_dict.items():
                vehicle_classes = [traci.vehicle.getVehicleClass(veh_id) for veh_id in vehicle_ids]
                vehicle_dict[lane_id] = vehicle_classes
                emergency_dict = {lane_id: 1 if "emergency" in vehicle_classes else 0 for lane_id, vehicle_classes in vehicle_dict.items()}
                emergencies[jID] = emergency_dict

        # for each jID, make an array of emergency values for each lane and then concat for all jIds
        emergency_array = np.array([list(emergencies[jID].values()) for jID in junction_ids])

        # for each jid in junction id, make an array of total vehicles in each lane and then concat
        vehicle_array = []
        for jId in junction_ids :
            junction_vehicle_numbers = [traci.lane.getLastStepVehicleNumber(lane_id) for lane_id in lane_ids[jId]]
            vehicle_array.append(junction_vehicle_numbers)

        vehicle_array = np.array(vehicle_array)


        print(emergency_array)


        




