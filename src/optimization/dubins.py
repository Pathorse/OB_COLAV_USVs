import numpy as np
from math import atan2, sqrt

import dubins



def dubins_path(wpts, theta_init=0, turning_radius=1, type='default'):

    # Initialize dubins path
    d_segments = []

    # Initialize total path length
    L = 0

    # Num wpts
    N = len(wpts)

    # Initial heading
    theta0 = theta_init

    for i in range(N - 1):

        # Current wpt
        c_wpt = wpts[i]

        # Next wpt
        n_wpt = wpts[i + 1]

        if type == 'adaptive':
            # Distance between wpts
            dx = n_wpt[0] - c_wpt[0]
            dy = n_wpt[1] - c_wpt[1]
            D = sqrt(dx**2 + dy**2)

            # Set adaptive turning radius
            turning_radius = D/4

        # Calculate desired ending heading
        if i + 1 == N - 1: # At last wpt

            # Extract values
            x   = n_wpt[0]
            y   = n_wpt[1]
            psi = theta1

            # Dubins initial and final config
            q0 = (c_wpt[0], c_wpt[1], theta1)
            q1 = (n_wpt[0], n_wpt[1], theta1)

            # Calculate dubins path between current and
            # next waypoint
            segment = dubins.shortest_path(q0, q1, turning_radius)

            # Find segment length
            L_segment = segment.path_length()

            # Update
            d_segments.append(segment)
            L += L_segment


        else: # Not at last wpt

            # Next next wpt
            nn_wpt   = wpts[i + 2]

            # Headings
            theta0 = atan2(n_wpt[1] - c_wpt[1], n_wpt[0] - c_wpt[0])
            theta1 = atan2(nn_wpt[1] - n_wpt[1], nn_wpt[0] - n_wpt[0])

            # Dubins initial and final config
            q0 = (c_wpt[0], c_wpt[1], theta0)
            q1 = (n_wpt[0], n_wpt[1], theta1)

            # Calculate dubins path between current and
            # next waypoint
            segment = dubins.shortest_path(q0, q1, turning_radius)

            # Find segment length
            L_segment = segment.path_length()

            # Update
            d_segments.append(segment)
            L += L_segment

    return d_segments, L



def dubins_sample_many(segments, step_size=0.2):

    # Initialize path
    d_path = []

    for segment in segments:

        # Extract configurations
        configurations, _ = segment.sample_many(step_size)

        for config in configurations:

            # Extract values
            x   = config[0]
            y   = config[1]
            psi = config[2]

            # Append to path
            d_path.append([x, y, psi])

    return d_path


def dubins_sample(segments, t):

    # Initialize total dist
    L = 0
    for segment in segments:

        # Extract segment length
        L_seg = segment.path_length()

        # Check for correct segment
        if t <= L + L_seg:

            # Extract config at distance traveled
            config = segment.sample(t - L)

            # Extract values
            x   = config[0]
            y   = config[1]
            psi = config[2]

            return [x, y, psi]

        # Add to total dist
        L += L_seg

    raise Exception('Failed to find dubins sample')


def dubins_generate_initial_guess(segments, L, time_steps):

    # Length stepsize
    dL =  L/time_steps

    # Sample state
    x_guess = np.vstack([dubins_sample(segments, dL * t) for t in range(time_steps + 1)])

    return x_guess
