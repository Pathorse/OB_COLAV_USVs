from environment.environment import Environment
from dynamics.ReVolt import ReVolt
from dynamics.simple_USV import SimpleUSV
from dynamics.hovercraft import Hovercraft
from objects.sphere import Sphere
from objects.polygon import Polygon, generatePolygon, plot_polygons_lines_and_points
from optimization.planner_1 import run_NLP, interpolate_rocket_state
from optimization.a_star import AStar
from optimization.dubins import dubins_path, dubins_sample_many, dubins_generate_initial_guess
from utilities.utilities import plot_usv_contour


from optimization.a_star_test import AStar_test

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

    start = np.array([10, 10])  # Start location
    goal  = np.array([430, 50]) # Goal location

    lb = [0, 0] # Lowerbound in x and y
    ub = [450, 200] # Upperbound in x and y

    # Testcase 2
    polygon_vertices = [
        np.array([[80,0],[80,30],[100,30],[100,0]]),
        np.array([[80,60],[80,200],[100,200],[100,60]]),
        np.array([[150,0],[250,90],[270,90],[170,0]]),
        np.array([[310,100],[310,200],[340,200],[340,100]])
        #np.array([[360,60],[310,110],[360,110],[430,60]]),
        #np.array([[0,200],[0,250],[500,250],[500,0]]),
        #np.array([[0,-150],[0,-100],[500,-100],[500,-150]])
    ]

    #polygon_vertices = [
    #  np.array([[40,40], [40, 60], [60,60], [60,40]])
    #]

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
    time_interval = 2.0
    time_steps    = 80

    # USV
    usv = ReVolt()
    #usv = SimpleUSV()
    #usv = Hovercraft()

    # Calculate
    x_opt, u_opt, x_guess = run_NLP(
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

    # Plot initial guess
    plt.plot(x_guess[:,0], x_guess[:,1], '-.')

    # Plot optimal trajectory
    plt.plot(x_opt[:,0], x_opt[:,1])

    # Plot usv
    plot_usv_contour(ax, x_opt, width=20, height=10, tip_height=32)

    #plt.axis('equal')
    plt.xlim(lb[0], ub[0])
    plt.ylim(lb[1], ub[1])
    plt.show()









def test_astar():

    start = np.array([0, 0])  # Start location
    goal  = np.array([320, 90]) # Goal location

    lb = [-50, -50] # Lowerbound in x and y
    ub = [350, 120] # Upperbound in x and y

    polygon_vertices = [
        np.array([[80,-20],[80,30],[100,30],[100,-20]]),
        np.array([[80,60],[80,120],[100,120],[100,60]]),
        np.array([[150,10],[150,65],[170,65],[170,10]])
    ]

    #polygon_vertices = [
    #  np.array([[40,40], [40, 60], [60,60], [60,40]])
    #]

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
    astar = AStar_test(start, goal, polygon_obstacles, lb, ub, resolution=1)
    path  = astar.plan()


# --------------------------------------------------------------
if __name__ == "__main__":
    main()
