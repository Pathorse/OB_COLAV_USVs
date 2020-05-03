from math import atan2

import dubins



def dubins_path(wpts, theta_init=0, turning_radius=1, step_size=0.2):

    # Initialize dubins path
    d_path = []

    # Num wpts
    N = len(wpts)

    # Initial heading
    theta0 = theta_init

    for i in range(N - 1):

        # Current wpt
        c_wpt = wpts[i]

        # Next wpt
        n_wpt = wpts[i + 1]

        # Calculate desired ending heading
        if i + 1 == N - 1: # At last wpt
            d_path.append(n_wpt)
        else:
            # Next next wpt
            nn_wpt   = wpts[i + 2]

            theta1 = atan2(nn_wpt[1] - n_wpt[1], nn_wpt[0] - n_wpt[0])

            # Dubins initial and final config
            q0 = (c_wpt[0], c_wpt[1], theta0)
            q1 = (n_wpt[0], n_wpt[1], theta1)

            # Calculate dubins path between current and
            # next waypoint
            path = dubins.shortest_path(q0, q1, turning_radius)
            configurations, _ = path.sample_many(step_size)

            for config in configurations:

                # Extract values
                x = config[0]
                y = config[1]

                # Append to path
                d_path.append([x, y])

            # Update heading
            theta0 = theta1


    return d_path

