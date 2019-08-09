from openmdao.api import Problem, Group
from heating import AerodynamicHeating
from flight_dynamics import FlightDynamics
from aerodynamics import Aerodynamics
from atmosphere import Atmosphere
import numpy as np

class ShuttleODE(Group):

    def initialize(self):
        self.options.declare("num_nodes", types=int)

    def setup(self):
        nn = self.options["num_nodes"]

        self.add_subsystem("atmosphere", subsys=Atmosphere(num_nodes=nn), promotes_inputs=["h"], promotes_outputs=["rho"])
        self.add_subsystem("aerodynamics", subsys=Aerodynamics(num_nodes=nn), promotes_inputs=["alpha", "v", "rho"], promotes_outputs=["lift", "drag"])
        self.add_subsystem("heating", subsys=AerodynamicHeating(num_nodes=nn), promotes_inputs=["rho", "v", "alpha"], promotes_outputs=["q"])
        self.add_subsystem("eom", subsys=FlightDynamics(num_nodes=nn), promotes_inputs=["beta", "gamma", "h", "psi", "theta", "v", "lift", "drag"], promotes_outputs=["hdot", "gammadot", "phidot", "psidot", "thetadot", "vdot"])

def test_shuttle_ode():
    prob = Problem()
    prob.model = ShuttleODE(num_nodes=5)

    prob.setup(check=False, force_alloc_complex=True)
    # prob["aerodynamics.alpha"] = 5
    # prob["heating.alpha"] = 5
    # prob["aerodynamics.v"] = 2500
    # prob["heating.v"] = 2500
    # prob["eom.v"] = 2500
    # prob["eom.gamma"] = -5*np.pi/180
    # prob["eom.theta"] = 40*np.pi/180 
    # prob["eom.beta"] = -5*np.pi/180
    # prob["eom.h"] = 80000
    # prob["atmosphere.h"] = 80000
    # prob["eom.psi"] = 45*np.pi/180

    prob.run_model()

    return(prob)

if __name__ == "__main__":

    prob = test_shuttle_ode()
    prob.check_partials(compact_print=True, method="cs")
    print(prob["psidot"])