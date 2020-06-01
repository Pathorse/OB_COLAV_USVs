import numpy as np
from numpy.linalg import inv

import pydrake.symbolic as sym



class ReVolt(object):

    def __init__(
            self,
            name="ReVolt",
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
        tau = np.array([u[0], 0, u[1]])*10**4

        # Define states
        x_n = x[0]
        y_n = x[1]
        psi = x[2]
        u   = x[3]
        v   = x[4]
        r   = x[5]

        m = sym if x.dtype == object else np

        # Rotation matrix about z-axis
        R = np.array([
            [m.cos(psi), -m.sin(psi), 0],
            [m.sin(psi), m.cos(psi), 0],
            [0,          0,          1]
        ])

        # Total Inertia matrix (M_RB + M_A)
        M = np.array([
            [263.93, 0.0,    0.0],
            [0.0,    306.44, 7.0],
            [0.0,    7.03,   322.15]
        ])

        # Coriolis-centripetal matrix
        C = np.array([
            [0,                  0,       -207.56*v + 7.00*r],
            [0,                  0,        250.07*u],
            [207.56*v - 7.00*r, -250.07*u, 0]
        ])

        # Damping matrix (linear)
        D = np.array([
            [50.66, 0,      0],
            [0,     601.45, 83.05],
            [0,     83.10,  268.17]
        ])


        # Define Position and Orientation vector
        eta = np.array([x_n, y_n, psi])

        # Define Linear and Angular Velocity vector
        ny  = np.array([u, v, r])

        # Derivatives
        eta_dot = R.dot(ny)
        ny_dot  = inv(M).dot(C.dot(ny) + D.dot(ny) + tau)

        # Concatenate to state vector derivative
        x_dot = np.concatenate((eta_dot, ny_dot))

        return x_dot

    # ----------------------------------------------------
    # Define discrete dynamics using Forward Euler
    # ----------------------------------------------------
    def discrete_dynamics(self, x, x_next, u, dt=0.1):
        x_dot  = self.continious_dynamics(x, u)
        residuals = x_next - x - dt * x_dot

        return residuals
