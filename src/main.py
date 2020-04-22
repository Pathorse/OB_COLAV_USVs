from dynamics.ReVolt import ReVolt
from dynamics.simple_USV import SimpleUSV
from dynamics.hovercraft import Hovercraft
from objects.sphere import Sphere
from objects.polygon import Polygon, generatePolygon, plot_polygons_lines_and_points
from path_planning.traj_opt import (interpolate_rocket_state, add_decision_variables, set_initial_and_terminal_position,
                                    set_dynamics, set_initial_guess, set_circle_obstacles, set_polygon_obstacles,
                                    set_binary, add_cost)
from utilities.utilities import plot_usv_contour


import numpy as np
import matplotlib as mpl
import matplotlib.patches
import matplotlib.pyplot as plt
import pypoman


# pydrake imports
from pydrake.all import (Variable, SymbolicVectorSystem, DiagramBuilder,
                         LogOutput, Simulator, ConstantVectorSource,
                         MathematicalProgram, Solve, SnoptSolver, PiecewisePolynomial)


def main():

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

    sphere_obstacles = [] # Initiate empty obstacle list
    sphere_obstacles.append(Sphere('Sphere_1', 'red', np.array([1000, 1000]), 300))
    #sphere_obstacles.append(Sphere('Sphere_1', 'red', np.array([1500, 1900]), 70))

    # ----------------------------------------------------
    # Optimization
    # ----------------------------------------------------

    # Numeric parameters
    time_interval = 0.4
    time_steps    = 100

    # USV
    #usv = ReVolt()
    usv = SimpleUSV()
    #usv = Hovercraft()

    # initialize optimization
    prog = MathematicalProgram()


    # optimization variables
    decision_variables = add_decision_variables(prog, usv.n_x, usv.n_u, len(polygon_obstacles), time_steps)

    # intial and terminal constraint
    set_initial_and_terminal_position(prog, start, goal, decision_variables)

    # discretized dynamics
    set_dynamics(prog, usv, decision_variables, time_interval, time_steps)

    # circle obstacle constraints
    set_circle_obstacles(prog, sphere_obstacles, decision_variables, time_steps)

    # polygon obstacle constraints
    #set_polygon_obstacles(prog, polygon_obstacles, decision_variables, start, goal, time_steps, len(polygon_obstacles))
    #set_binary(prog, decision_variables, time_steps)

    # cost
    add_cost(prog, decision_variables, time_interval, time_steps)

    # initial guess
    set_initial_guess(prog, start, goal, decision_variables, time_interval, time_steps)

    # solve mathematical program
    solver = SnoptSolver()
    result = solver.Solve(prog)

    # assert the solution
    assert result.is_success()

    # retrive optimal solution
    decision_variables_opt = [result.GetSolution(v) for v in decision_variables]

    x_opt, u_opt = decision_variables_opt[:2]



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
    #for polygon in polygon_obstacles:
    #    polygon.plot()
    for circle in sphere_obstacles:
        circle.plot_contour()

    # Plot interpolated trajectory
    #state_guess = interpolate_rocket_state(start, goal)
    #plt.plot(state_guess[:,0], state_guess[:,1])

    # Plot optimal trajectory
    plt.plot(x_opt[:,0], x_opt[:,1])

    # Plot usv
    plot_usv_contour(ax, x_opt, width=20, height=10, tip_height=32)

    plt.axis('equal')
    plt.show()
  














   
# --------------------------------------------------------------
if __name__ == "__main__":
    main()
