from environment.environment import Environment
from dynamics.ReVolt import ReVolt
from dynamics.simple_USV import SimpleUSV
from dynamics.hovercraft import Hovercraft
from objects.sphere import Sphere
from objects.polygon import Polygon, generatePolygon, plot_polygons_lines_and_points
from optimization.planner import run_NLP, interpolate_rocket_state
from optimization.a_star import AStar
from optimization.dubins import dubins_path, dubins_sample_many, dubins_generate_initial_guess
from utilities.utilities import plot_usv_contour



import numpy as np
import matplotlib as mpl
import matplotlib.patches
import matplotlib.pyplot as plt
import pypoman
import dubins

import pdb # TODO remove

# Drake imports
from pydrake.all import (Variable, SymbolicVectorSystem, DiagramBuilder,
                         LogOutput, Simulator, ConstantVectorSource,
                         MathematicalProgram, Solve, SnoptSolver, OsqpSolver,
                         PiecewisePolynomial)



def main():

    # ----------------------------------------------------
    # Setup Simulation Environment
    # ----------------------------------------------------

    start = np.array([0, 0])  # Start location
    goal  = np.array([1000, 1000]) # Goal location

    lb = [-100, -100] # Lowerbound in x and y
    ub = [1100, 1100] # Upperbound in x and y

    polygon_vertices = [ # Consists of Polygon vertices on form [(x1,y1), (x2,y2), ...]
        #generatePolygon( ctrX=500, ctrY=1250, aveRadius=700, irregularity=0.0, spikeyness=0.0, numVerts=7),
        generatePolygon( ctrX=700, ctrY=950, aveRadius=100, irregularity=0.0, spikeyness=0.0, numVerts=4),
        generatePolygon( ctrX=500, ctrY=500, aveRadius=200, irregularity=0.0, spikeyness=0.0, numVerts=5),
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

    env = Environment(obstacles=polygon_obstacles, lb=lb, ub=ub) # Environment


    sphere_obstacles = [] # Initiate empty obstacle list
    sphere_obstacles.append(Sphere('Sphere_1', 'red', np.array([1000, 1000]), 300))
    sphere_obstacles.append(Sphere('Sphere_1', 'red', np.array([1500, 1900]), 70))

    # ----------------------------------------------------
    # Optimization
    # ----------------------------------------------------

    # Numeric parameters
    time_interval = 0.2
    time_steps    = 100

    # USV
    #usv = ReVolt()
    usv = SimpleUSV()
    #usv = Hovercraft()

    # Calculate
    x_opt, u_opt = run_NLP(
        env,
        usv,
        start,
        goal,
        lb,
        ub,
        time_interval,
        time_steps
    )


    # ----------------------------------------------------
    # Plotting
    # ----------------------------------------------------
   
    plt.figure()
    ax = plt.gca()
  
    # Plot start and goal
    plt.plot(start[0], start[1], ".b", markersize=10)
    plt.plot(goal[0], goal[1], "*r", markersize=10)

    # Plot obstacles
    #plot_polygons_lines_and_points(fig, blue_polygons=obstacles)
    for polygon in polygon_obstacles:
        polygon.plot()
    #for circle in sphere_obstacles:
    #    circle.plot_contour()

    # Plot interpolated trajectory
    #state_guess = interpolate_rocket_state(start, goal)
    #plt.plot(state_guess[:,0], state_guess[:,1])

    # Plot environment
    #env.draw(ax)

    # Plot optimal trajectory
    plt.plot(x_opt[:,0], x_opt[:,1])

    # Plot usv
    plot_usv_contour(ax, x_opt, width=20, height=10, tip_height=32)

    plt.axis('equal')
    plt.show()














# --------------------------------------------------------------
if __name__ == "__main__":
    main()
