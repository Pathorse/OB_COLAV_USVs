import numpy as np




def astar_demo():

    # ----------------------------------------------------
    # Setup Simulation Environment
    # ----------------------------------------------------

    start = [0, 0]  # Start location
    goal  = [2000, 2000] # Goal location

    lb = [-100, -100] # Lowerbound in x and y
    ub = [2100, 2100] # Upperbound in x and y

    polygon_vertices = [ # Consists of Polygon vertices on form [(x1,y1), (x2,y2), ...]
        generatePolygon( ctrX=500, ctrY=1250, aveRadius=700, irregularity=0.0, spikeyness=0.0, numVerts=7),
        #generatePolygon( ctrX=1650, ctrY=1650, aveRadius=150, irregularity=0.0, spikeyness=0.0, numVerts=5),
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
    # Plotting
    # ----------------------------------------------------

    plt.figure()

    # Plot obstacles
    for polygon in polygon_obstacles:
        polygon.plot()

    plt.axis('equal')
    plt.show()


if __name__ == "__main__":
    astar_demo()
