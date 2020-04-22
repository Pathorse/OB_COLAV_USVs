import numpy as np
from numpy.linalg import inv

import pydrake.symbolic as sym



class SimpleUSV(object):

    def __init__(
            self,
            name="SimpleUSV",
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
        tau = np.array([u[0], 0, u[1]])*10**3

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
            [m.cos(psi), m.sin(psi), 0.0],
            [m.sin(psi), m.cos(psi), 0.0],
            [0.0,        0.0,        1.0]
        ])

        # Total Inertia matrix (M_RB + M_A)
        m1 = 120.0 * 10**(3)
        m2 = 172.9 * 10**(3)
        m3 = 636.0 * 10**(5)
        M = np.array([
            [m1,   0.0,    0.0],
            [0.0,  m2,     0.0],
            [0.0,  0.0,    m3 ]
        ])

        # Coriolis-centripetal matrix
        C = np.array([
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0]
        ])

        # Damping matrix (linear)
        d1 = 215.0 * 10**(2)
        d2 = 97.0  * 10**(3)
        d3 = 802.0 * 10**(4)
        D = np.array([
            [d1,    0.0,    0.0],
            [0.0,   d2,     0.0],
            [0.0,   0.0,    d3 ]
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
