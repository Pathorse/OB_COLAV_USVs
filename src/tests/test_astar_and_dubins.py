


def astar_dubins_demo():

    # ----------------------------------------------------
    # Setup Simulation Environment
    # ----------------------------------------------------

    start = [0, 0]  # Start location
    goal  = [19, 19] # Goal location

    lb = [0, 0] # Lowerbound in x and y
    ub = [20, 20] # Upperbound in x and y

    polygon_vertices = [ # Consists of Polygon vertices on form [(x1,y1), (x2,y2), ...]
        generatePolygon( ctrX=10, ctrY=10, aveRadius=5, irregularity=0.0, spikeyness=0.0, numVerts=7),
        generatePolygon( ctrX=17, ctrY=16, aveRadius=3, irregularity=0.0, spikeyness=0.0, numVerts=5),
        generatePolygon( ctrX=10, ctrY=17.5, aveRadius=2, irregularity=0.0, spikeyness=0.0, numVerts=5),
        #generatePolygon( ctrX=1600, ctrY=400, aveRadius=300, irregularity=0.0, spikeyness=0.0, numVerts=5),
        #generatePolygon( ctrX=1700, ctrY=1200, aveRadius=100, irregularity=0.0, spikeyness=0.0, numVerts=7),
    ]


    polygon_obstacles = [] # Initiate empty obstacle list
    for i in range(len(polygon_vertices)): # Fill obstacles with polygons
        polygon_obstacles.append(
            Polygon(
                polygon_vertices[i],
                f'Obstacle_{i}',
                'darkorange'
            )
        )


    # ----------------------------------------------------
    # A*
    # ----------------------------------------------------

    astar = AStar(start, goal, polygon_obstacles, lb, ub, resolution=0.25)


    path = astar.plan()

    d_segments, L = dubins_path(path, turning_radius=1, type='adaptive')

    d_path = dubins_sample_many(d_segments)

    x_guess = dubins_generate_initial_guess(d_segments, L, time_steps=100)

    print(x_guess)
    print(x_guess.shape)


    # ----------------------------------------------------
    # Plotting
    # ----------------------------------------------------

    plt.figure()
    ax = plt.gca()

    # Plot start and goal
    plt.plot(start[0], start[1], ".b", markersize=10)
    plt.plot(goal[0], goal[1], "*r", markersize=10)

    # Plot obstacles
    for polygon in polygon_obstacles:
        polygon.plot()

    # Plot path
    x_p = []
    y_p = []

    for x, y in path:
        x_p.append(x)
        y_p.append(y)

    plt.plot(x_p, y_p, 'r-.')

    # Plot dubins path
    x_p   = []
    y_p   = []
    psi_p = []

    for x, y, psi in d_path:
        x_p.append(x)
        y_p.append(y)
        psi_p.append(psi)

    x_trj = np.array([x_p, y_p, psi_p]).T

    plot_usv_contour(ax, x_trj, width=0.2, height=0.1, tip_height=0.3)

    plt.plot(x_p, y_p, 'b')

    plt.axis('equal')
    plt.show()
