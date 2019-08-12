import numpy as np 
import matplotlib.pyplot as plt 
from openmdao.api import Problem, Group, ScipyOptimizeDriver, SqliteRecorder, CaseReader, IndepVarComp
from dymos import Trajectory, GaussLobatto, Phase, Radau
from shuttle_ode import ShuttleODE

prob = Problem(model=Group())

traj = prob.model.add_subsystem("traj", Trajectory())

phase0 = Phase(ode_class=ShuttleODE, transcription=Radau(num_segments=20, order=3))
traj.add_phase(name="phase0", phase=phase0)

phase0.set_time_options(fix_initial=True, units="s", duration_ref=200)#, duration_ref=2000, duration_bounds=(50, 3000)

phase0.set_state_options("h", fix_initial=True, fix_final=True, units="ft", rate_source="hdot", targets=["h"], lower=0)#, ref=260000, defect_ref=260000, ref0=80000
phase0.set_state_options("gamma", fix_initial=True, fix_final=True, units="rad", rate_source="gammadot", targets=["gamma"], lower=-89.*np.pi/180, upper=89.*np.pi/180)
phase0.set_state_options("phi", fix_initial=True, fix_final=False, units="rad", rate_source="phidot", lower=0, upper=89.*np.pi/180)
phase0.set_state_options("psi", fix_initial=True, fix_final=False, units="rad", rate_source="psidot", targets=["psi"], lower=0, upper=90.*np.pi/180)
phase0.set_state_options("theta", fix_initial=True, fix_final=False, units="rad", rate_source="thetadot", targets=["theta"], lower=-89.*np.pi/180, upper=89.*np.pi/180)
phase0.set_state_options("v", fix_initial=True, fix_final=True, units="ft/s", rate_source="vdot", targets=["v"], lower=0)#, ref=25600, defect_ref=25600, ref0=2500

phase0.add_control("alpha", units="rad", opt=True, lower=-np.pi/2, upper=np.pi/2, targets=["alpha"])
phase0.add_control("beta", units="rad", opt=True, lower=-89*np.pi/180, upper=1*np.pi/180, targets=["beta"])

phase0.add_path_constraint("q", lower=0, upper=70, units="Btu/ft**2/s", ref=70)#

phase0.add_objective("theta", loc="final", ref=-1)

prob.driver = ScipyOptimizeDriver()
prob.driver.declare_coloring()
prob.driver.options["maxiter"] = 100

prob.setup(check=True)

prob.set_val("traj.phase0.states:h", phase0.interpolate(ys=[260000, 80000], nodes="state_input"), units="ft")
prob.set_val("traj.phase0.states:gamma", phase0.interpolate(ys=[-1*np.pi/180, -5*np.pi/180], nodes="state_input"), units="rad")
prob.set_val("traj.phase0.states:phi", phase0.interpolate(ys=[0, 75*np.pi/180], nodes="state_input"), units="rad")
prob.set_val("traj.phase0.states:psi", phase0.interpolate(ys=[90*np.pi/180, 10*np.pi/180], nodes="state_input"), units="rad")
prob.set_val("traj.phase0.states:theta", phase0.interpolate(ys=[0, 25*np.pi/180], nodes="state_input"), units="rad")
prob.set_val("traj.phase0.states:v", phase0.interpolate(ys=[25600, 2500], nodes="state_input"), units="ft/s")
prob.set_val("traj.phase0.t_initial", 0, units="s")
prob.set_val("traj.phase0.t_duration", 2000, units="s")
prob.set_val("traj.phase0.controls:alpha", phase0.interpolate(ys=[17.4*np.pi/180, 17.4*np.pi/180], nodes="control_input"), units="rad")
prob.set_val("traj.phase0.controls:beta", phase0.interpolate(ys=[-75*np.pi/180, 0*np.pi/180], nodes="control_input"), units="rad")

recorder = SqliteRecorder("reentry.sql")
prob.driver.add_recorder(recorder)

prob.run_driver()
# prob.run_model()

sim_out = traj.simulate()

prob.cleanup()

case_reader = CaseReader("reentry.sql")

driver_cases = case_reader.list_cases("driver")
last_case = case_reader.get_case(driver_cases[-1])
final_constraints = last_case.get_constraints()
final_q = final_constraints["traj.phase0.path_constraints.path:q"]

plt.figure(0)
plt.plot(prob.get_val("traj.phase0.timeseries.time", units="s"), prob.get_val("traj.phase0.timeseries.controls:alpha", units="deg"), "ro", label="Solution")
plt.plot(sim_out.get_val("traj.phase0.timeseries.time", units="s"), sim_out.get_val("traj.phase0.timeseries.controls:alpha", units="deg"), "b-", label="Simulation")
plt.title("Angle of Attack over Time")
plt.xlabel("Time (s)")
plt.ylabel("Angle of Attack (degrees)")
plt.legend()

plt.figure(1)
plt.plot(prob.get_val("traj.phase0.timeseries.time", units="s"), prob.get_val("traj.phase0.timeseries.controls:beta", units="deg"), "ro", label="Solution")
plt.plot(sim_out.get_val("traj.phase0.timeseries.time", units="s"), sim_out.get_val("traj.phase0.timeseries.controls:beta", units="deg"), "b-", label="Simulation")
plt.title("Bank Angle over Time")
plt.xlabel("Time points")
plt.ylabel("Bank Angle (degrees)")
plt.legend()

plt.figure(2)
plt.plot(prob.get_val("traj.phase0.timeseries.time", units="s"), prob.get_val("traj.phase0.timeseries.states:h", units="ft"), "ro", label="Solution")
plt.plot(sim_out.get_val("traj.phase0.timeseries.time", units="s"), sim_out.get_val("traj.phase0.timeseries.states:h", units="ft"), "b-", label="Simulation")
plt.title("Altitude over Time")
plt.xlabel("Time (s)")
plt.ylabel("Altitude (feet)")
plt.legend()

plt.figure(3)
plt.plot(prob.get_val("traj.phase0.timeseries.time", units="s"), prob.get_val("traj.phase0.timeseries.states:gamma", units="deg"), "ro", label="Solution")
plt.plot(sim_out.get_val("traj.phase0.timeseries.time", units="s"), sim_out.get_val("traj.phase0.timeseries.states:gamma", units="deg"), "b-", label="Simulation")
plt.title("Flight Path Angle over Time")
plt.xlabel("Time (s)")
plt.ylabel("Flight Path Angle (degrees)")
plt.legend()

plt.figure(4)
plt.plot(prob.get_val("traj.phase0.timeseries.time", units="s"), prob.get_val("traj.phase0.timeseries.states:phi", units="deg"), "ro", label="Solution")
plt.plot(sim_out.get_val("traj.phase0.timeseries.time", units="s"), sim_out.get_val("traj.phase0.timeseries.states:phi", units="deg"), "b-", label="Simulation")
plt.title("Longitude over Time")
plt.xlabel("Time (s)")
plt.ylabel("Longitudinal Angle (degrees)")
plt.legend()

plt.figure(5)
plt.plot(prob.get_val("traj.phase0.timeseries.time", units="s"), prob.get_val("traj.phase0.timeseries.states:psi", units="deg"), "ro", label="Solution")
plt.plot(sim_out.get_val("traj.phase0.timeseries.time", units="s"), sim_out.get_val("traj.phase0.timeseries.states:psi", units="deg"), "b-", label="Simulation")
plt.title("Azimuth over Time")
plt.xlabel("Time (s)")
plt.ylabel("Azimuthal Angle (degrees)")
plt.legend()

plt.figure(6)
plt.plot(prob.get_val("traj.phase0.timeseries.time", units="s"), prob.get_val("traj.phase0.timeseries.states:theta", units="deg"), "ro", label="Solution")
plt.plot(sim_out.get_val("traj.phase0.timeseries.time", units="s"), sim_out.get_val("traj.phase0.timeseries.states:theta", units="deg"), "b-", label="Simulation")
plt.title("Latitude over Time")
plt.xlabel("Time (s)")
plt.ylabel("Latitudinal Angle (degrees)")
plt.legend()

plt.figure(7)
plt.plot(prob.get_val("traj.phase0.timeseries.time", units="s"), prob.get_val("traj.phase0.timeseries.states:v", units="ft/s"), "ro", label="Solution")
plt.plot(sim_out.get_val("traj.phase0.timeseries.time", units="s"), sim_out.get_val("traj.phase0.timeseries.states:v", units="ft/s"), "b-", label="Simulation")
plt.title("Velocity over Time")
plt.xlabel("Time (s)")
plt.ylabel("Velocity (ft/s)")
plt.legend()

plt.figure(8)
plt.plot(np.arange(len(final_q)), final_q)
plt.title("Heating rate over Time")
plt.xlabel("Time points")
plt.ylabel("Heating Rate (BTU/ft**2/s")

plt.show()

# print(prob.get_val("traj.phase0.collocation_constraint.defects:v"))
print("final time is: ", prob.get_val("traj.phase0.timeseries.time")[-1])
print("final h is: ", prob.get_val("traj.phase0.timeseries.states:h")[-1])
print("final gamma is: ", prob.get_val("traj.phase0.timeseries.states:gamma", units="deg")[-1])
print("final v is: ", prob.get_val("traj.phase0.timeseries.states:v")[-1])