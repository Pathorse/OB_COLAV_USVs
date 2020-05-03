import dubins


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
    d_path = dubins_path(wpts, turning_radius=1.5)

    print(len(d_path))


    # Plotting
    xp = []
    yp = []

    for x, y in d_path:
        xp.append(x)
        yp.append(y)

    plt.figure()
    plt.plot(xp,yp)
    plt.axis('equal')
    plt.show()
