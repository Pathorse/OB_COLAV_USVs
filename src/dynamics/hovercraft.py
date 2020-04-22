import numpy as np
from numpy.linalg import inv

import pydrake.symbolic as sym



class Hovercraft(object):

    def __init__(
            self,
            name="Hovercraft",
            n_x=6,
            n_u=2
    ):
        # Declare name
        self.name = name

        # Declare number of state variables and input variables
        self.n_x  = n_x
        self.n_u  = n_u

    # ----------------------------------------------------
    # Define continious dynamics
    # ----------------------------------------------------
    def continious_dynamics(self, x, u):
        # x   = [x, y, psi, u, v, r] = [posx, posy, heading, surge vel, sway vel, yaw rate]
        # u = [tau_u, tau_r] = [surge inp, yaw rate inp]

        # Define inputs
        tau = np.array([u[0], 0, u[1]])*10**0

        # Define states
        x_n = x[0]
        y_n = x[1]
        psi = x[2]
        u   = x[3]
        v   = x[4]
        r   = x[5]

        m = sym if x.dtype == object else np

        # Funection of elements of the inertia matrix and
        # hydrodynamic damping matrix
        beta = 1/2
       
        # Define model
        x_n_dot = u * m.cos(psi) - v * m.sin(psi)
        y_n_dot = u * m.sin(psi) + v * m.cos(psi)
        psi_dot = r
        u_dot   = v * r + tau[0]
        v_dot   = - u * r - beta * v
        r_dot   = tau[2]

        #  State vector derivative
        x_dot = np.array([
            x_n_dot,
            y_n_dot,
            psi_dot,
            u_dot,
            v_dot,
            r_dot
        ])

        return x_dot

    # ----------------------------------------------------
    # Define discrete dynamics using Forward Euler
    # ----------------------------------------------------
    def discrete_dynamics(self, x, x_next, u, dt=0.1):
        x_dot  = self.continious_dynamics(x, u)
        residuals = x_next - x - dt * x_dot

        return residuals
