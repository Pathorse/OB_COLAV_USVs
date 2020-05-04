import dubins
import math
import matplotlib.pyplot as plt


def dubins_demo():

    # Waypoints
    wpts = [
        [0,0],
        [5,5],
        [8,10],
        [15,12],
        [20,20]
    ]

    # Calculate dubins path
    d_segments, L = dubins_path(wpts, turning_radius=1.5)
    d_path = dubins_sample_many(d_segments)

    x_guess = dubins_generate_initial_guess(d_segments, L, time_steps=100)

    print(x_guess)
    print(x_guess.shape)

    sg = interpolate_rocket_state([0,0], [20,20])

    #print(sg)
    #print(sg.shape)
    # Plot dubins path
    x_p   = []
    y_p   = []
    psi_p = []

    for x, y, psi in d_path:
        x_p.append(x)
        y_p.append(y)
        psi_p.append(psi)

    x_trj = np.array([x_p, y_p, psi_p]).T

    plt.figure()
    ax = plt.gca()
    plot_usv_contour(ax, x_trj, width=0.2, height=0.1, tip_height=0.3)
    plt.plot(x_p, y_p, 'b')
    plt.axis('equal')
    plt.show()


def test_readme_demo():
    q0 = (0, 0, 0)
    q1 = (10, 10, 0)
    turning_radius = 2.0
    step_size = 0.2

    path = dubins.shortest_path(q0, q1, turning_radius)
    configurations, _ = path.sample_many(step_size)

    print(path.path_length())
    print(path.sample(14.5))

    # Plot dubins path
    x_p   = []
    y_p   = []
    psi_p = []

    for x, y, psi in configurations:
        x_p.append(x)
        y_p.append(y)
        psi_p.append(psi)

    plt.figure()
    plt.plot(x_p, y_p, 'b')
    plt.axis('equal')
    plt.show()


if __name__ == "__main__":
    dubins_demo()
