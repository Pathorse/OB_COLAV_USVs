
    # ----------------------------------------------------
    # Setup Simulation Environment
    # ----------------------------------------------------

    start = np.array([0, 0])  # Start location
    goal  = np.array([2000, 2000]) # Goal location

    bounds = np.array([-100, 2100]) # Bounds in both x and y

    polygon_vertices = [ # Consists of Polygon vertices on form [(x1,y1), (x2,y2), ...]
        generatePolygon( ctrX=500, ctrY=1250, aveRadius=700, irregularity=0.0, spikeyness=0.0, numVerts=7),
        generatePolygon( ctrX=1650, ctrY=1650, aveRadius=150, irregularity=0.0, spikeyness=0.0, numVerts=5),
        generatePolygon( ctrX=1600, ctrY=400, aveRadius=300, irregularity=0.0, spikeyness=0.0, numVerts=5),
        generatePolygon( ctrX=1700, ctrY=1200, aveRadius=100, irregularity=0.0, spikeyness=0.0, numVerts=7),
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

    pdb.set_trace()
    sphere_obstacles = [] # Initiate empty obstacle list
    sphere_obstacles.append(Sphere('Sphere_1', 'red', np.array([1000, 1000]), 300))
    #sphere_obstacles.append(Sphere('Sphere_1', 'red', np.array([1500, 1900]), 70))
