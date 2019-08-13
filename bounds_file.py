#This file only needs to be run once, but is left in the repository
#for reference. ubs is a dictionary of the upper bounds on the states,
#controls, and time duration for the dymos model with which this
#file corresponds. lbs is the same dictionary but for lower bounds.
#The file saves these dictionaries as pickle files, and optionally 
#(currently commented out) prints out the dictionaries. These 
#dictionaries are accepted as inputs to the autoscale method in
#dymos (method is currently developmental).

import pickle
import numpy as np

lbs = {}
ubs = {}

ubs["traj.phases.phase0.time_extents.t_duration"] = 2000.
ubs["traj.phases.phase0.indep_states.states:h"] = 260000.
ubs["traj.phases.phase0.indep_states.states:gamma"] = -1.*np.pi/180.
ubs["traj.phases.phase0.indep_states.states:phi"] = 75.*np.pi/180.
ubs["traj.phases.phase0.indep_states.states:psi"] = 90.*np.pi/180.
ubs["traj.phases.phase0.indep_states.states:theta"] = 25.*np.pi/180.
ubs["traj.phases.phase0.indep_states.states:v"] = 25600.
ubs["traj.phases.phase0.control_group.indep_controls.controls:alpha"] = 17.4*np.pi/180.
ubs["traj.phases.phase0.control_group.indep_controls.controls:beta"] = 0.1*np.pi/180.

lbs["traj.phases.phase0.time_extents.t_duration"] = .01
lbs["traj.phases.phase0.indep_states.states:h"] = 80000.
lbs["traj.phases.phase0.indep_states.states:gamma"] = -5.*np.pi/180.
lbs["traj.phases.phase0.indep_states.states:phi"] = 0.
lbs["traj.phases.phase0.indep_states.states:psi"] = 10.*np.pi/180.
lbs["traj.phases.phase0.indep_states.states:theta"] = 0.
lbs["traj.phases.phase0.indep_states.states:v"] = 2500.
lbs["traj.phases.phase0.control_group.indep_controls.controls:alpha"] = 17.4*np.pi/180.
lbs["traj.phases.phase0.control_group.indep_controls.controls:beta"] = -75.*np.pi/180.

with open("lower_bounds_info.pickle", "wb") as file:
    pickle.dump(lbs, file, protocol=pickle.HIGHEST_PROTOCOL)
with open("upper_bounds_info.pickle", "wb") as file:
    pickle.dump(ubs, file, protocol=pickle.HIGHEST_PROTOCOL)

# with open("lower_bounds_info.pickle", "rb") as file:
#     file1 = pickle.load(file)

# with open("upper_bounds_info.pickle", "rb") as file:
#     file2 = pickle.load(file)
# print("lower bounds are: ")
# print(file1)
# print("upper bounds are: ")
# print(file2)