import numpy as np




def astar_demo():

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

    print('path:', path)



    # ----------------------------------------------------
    # Plotting
    # ----------------------------------------------------

    plt.figure()

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

    plt.plot(x_p, y_p)

    plt.axis('equal')
    plt.show()


if __name__ == "__main__":
    astar_demo()
