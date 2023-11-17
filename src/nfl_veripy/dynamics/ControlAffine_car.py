import numpy as np
from nfl_veripy.utils.mpc import control_mpc
from scipy.linalg import solve_discrete_are

from .Dynamics import DiscreteTimeDynamics

class ControlAffineCar(DiscreteTimeDynamics):
    def __init__(self):
        self.continuous_time = False
        dt = 0.01

        #Define constants 
        a = 1.2
        b = 1.4
        Iz = 2300
        m = 1700
        delta = 0.0872665 #5 degree steer angle

        At = np.array([[0,0,0], [0,0,1], [0, -1, 0]])
        Bt = np.array([[a*np.cos(delta)/Iz, a*np.sin(delta)/Iz, -b/Iz, 0],
                       [-np.sin(delta)/m, np.cos(delta)/m, 0, 1/m],
                       [np.cos(delta)/m, np.sin(delta)/m, 1/m, 0]])
        Ct = np.array([0.0, 0.0, 0.0]).T #No observation matrix
        u_limits = None

        super().__init__(At=At, bt=Bt, ct=Ct, dt=dt, u_limits=u_limits)

    def control_mpc(self, x0):
        #Not an MPC controller, just copying format 
        #For now, use constant control 

        return [20, 20, 20, 20]