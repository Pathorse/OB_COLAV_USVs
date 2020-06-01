import numpy as np

from timeit import default_timer

#from .a_star import AStar
from .a_star_revised import AStar
from .a_star_test import AStar_test
from .dubins import dubins_path, dubins_generate_initial_guess

import pdb # TODO Remove when finished

# Pydrake imports
from pydrake.all import (Variable, SymbolicVectorSystem, DiagramBuilder,
                         LogOutput, Simulator, ConstantVectorSource,
                         MathematicalProgram, Solve, SnoptSolver, PiecewisePolynomial,
                         OsqpSolver, eq, le, ge, gt, lt)
from pydrake.solvers import branch_and_bound



# ----------------------------------------------------
# Add decision variables to the optimization problem
# ----------------------------------------------------
def add_decision_variables(prog, obstacles, n_x, n_u, time_steps):

    # Optimization variables
    x = prog.NewContinuousVariables(rows=time_steps+1, cols=n_x, name='x')
    u = prog.NewContinuousVariables(rows=time_steps, cols=n_u, name='u')

    # Initialize list of lambdas
    lambdas = []

    # Number of obstacles
    N_obst = len(obstacles)

    for i in range(N_obst):

        # Extract polygon
        polygon = obstacles[i]

        # Extract b
        b = polygon.b

        # Number of edges in the polygon
        n_edges = len(b)

        # lambda variable for obstacle i
        l = prog.NewContinuousVariables(rows=time_steps + 1, cols=n_edges, name=f'lambda_{i}')

        # Add to list of lambdas
        lambdas.append(l)

    return x, u, lambdas


# ----------------------------------------------------
# Set initial and final condition for the
# optimization problem
# ----------------------------------------------------
def set_initial_and_terminal_position(prog, start, goal, decision_variables):

    # Unpack state and input
    x, u = decision_variables[:2]

    # Initial state
    x_0 = np.concatenate((start, np.array([0, 0, 0, 0])))

    # Enforce initial state
    #prog.AddLinearConstraint(eq(x[0], x_0))
    prog.AddLinearConstraint(eq(x[0,:2], start))

    # Enforce final position
    prog.AddLinearConstraint(eq(x[-1,:2], goal))

    # Enforce zero final velocity and yaw rate
    prog.AddLinearConstraint(eq(x[-1,3:], np.array([0, 0, 0])))


# ----------------------------------------------------
# Set an initial guess in order to perform a
# warm-start of the optimization problem
# ----------------------------------------------------
def set_initial_guess(prog, env, start, goal, decision_variables, time_interval, time_steps):

    # Unpack state and input
    x, u = decision_variables[:2]

    # Extract obstacles and bounds
    polygon_obstacles = env.obstacles
    lb                = env.lb
    ub                = env.ub

    # Interpolated initial guess
    #p_guess = interpolate_rocket_state(start, goal, time_interval, time_steps)

    # Calculate A* plan
    astar = AStar(start, goal, polygon_obstacles, lb, ub, resolution=4)
    #astar = AStar_test(start, goal, polygon_obstacles, lb, ub, resolution=4)
    path  = astar.plan()

    # Calculate Dubins Segments
    d_segments, L = dubins_path(path, type='adaptive')   # NOTE: Run this for adaptive turning rad
    #d_segments, L = dubins_path(path, turning_radius=3) # NOTE: Run this for pre-set turning rad

    # Generate initial guess in [x, y, psi]
    x_guess = dubins_generate_initial_guess(d_segments, L, time_steps)

    # Set initial guess
    #prog.SetInitialGuess(x[:,:2], p_guess[:,:2])
    prog.SetInitialGuess(x[:,:3], x_guess)

    return x_guess


# ----------------------------------------------------
# Set state and input constraints for the
# optimization problem
# -----------------------:w
# -----------------------------
def set_state_and_input_constraints(prog, env, usv, decision_variables, time_interval, time_steps):

    # Unpack state and input
    x, u = decision_variables[:2]

    # State bounds
    x_lb = np.array([- np.Inf]*usv.n_x)
    #x_lb = np.array([- np.Inf, - np.Inf, 0, - np.Inf, - np.Inf, - np.Inf])
    x_ub = np.array([np.Inf]*usv.n_x)
    #x_ub = np.array([np.Inf, np.Inf, 2*np.pi, np.Inf, np.Inf, np.Inf])

    # Input bounds
    u_lb = np.array([-np.Inf]*usv.n_u)
    #u_lb = np.array([-10**2]*usv.n_u)
    #u_lb = np.array([-50, -0.02])
    u_ub = np.array([np.Inf]*usv.n_u)
    #u_ub = np.array([10**2]*usv.n_u)
    #u_ub = np.array([50, 0.02])

    # Add Constraints
    for t in range(1, time_steps + 1):
        prog.AddLinearConstraint(x[t], lb=x_lb, ub=x_ub)
        prog.AddLinearConstraint(u[t - 1], lb=u_lb, ub=u_ub)


# ----------------------------------------------------
# Set the USV dynamics as a constraint for the
# optimization problem
# ----------------------------------------------------
def set_dynamics(prog, usv, decision_variables, time_interval, time_steps):

    # Unpack state and input
    x, u = decision_variables[:2]

    # Enforce discrete dynamics
    for t in range(time_steps):
        residuals = usv.discrete_dynamics(x[t], x[t+1], u[t], time_interval)
        for residual in residuals:
            prog.AddConstraint(residual == 0)


# ----------------------------------------------------
# Set an initial guess in order to perform a
# warm-start of the optimization problem
# ----------------------------------------------------
def set_polygon_obstacles(prog, obstacles, decision_variables, time_steps, safety_dist=3):

    # Unpack state and input
    x, u, lambdas = decision_variables

    # Number of obstacles
    N_obst = len(obstacles)

    # Enforce constraints
    for t in range(time_steps):

        # Extract decision variable x
        p = x[t,:2]
       
        for i in range(N_obst):

            # Extract decision variable lambda
            # for the current obstacle
            l = lambdas[i][t]

            # Extract polygon
            polygon = obstacles[i]

            # Extract halfspace matrices
            A = polygon.A
            b = polygon.b

            # Residual
            residual = (A.dot(p) - b).T.dot(l)

            #pdb.set_trace()
           
            # Add constraints
            prog.AddConstraint( (residual >= safety_dist) )

            prog.AddConstraint(
                ( (A.T.dot(l)).dot(A.T.dot(l)) <= 1 )
            )

            prog.AddLinearConstraint( ge(l, 0.01) )



# ----------------------------------------------------
# Add cost to the optimization problem
# ----------------------------------------------------
def add_cost(prog, decision_variables, time_interval, time_steps):

    # Unpack state and input
    x, u = decision_variables[:2]

    # minimize fuel consumption
    for t in range(time_steps):
        prog.AddQuadraticCost(time_interval*u[t].dot(u[t]))
        #prog.AddLinearCost(u[t,0] + u[t,1])


# ----------------------------------------------------
# Setup Simulation Environment
# ----------------------------------------------------
def run_NLP(env, usv, start, goal, lb, ub, time_interval, time_steps):

    # Start timer
    NLP_start_time = default_timer()
   
    # initialize optimization
    prog = MathematicalProgram()

    # optimization variables
    decision_variables = add_decision_variables(prog, env.obstacles, usv.n_x, usv.n_u, time_steps)

    # intial and terminal constraint
    set_initial_and_terminal_position(prog, start, goal, decision_variables)

    # initial guess
    x_guess = set_initial_guess(prog, env, start, goal, decision_variables, time_interval, time_steps)
    #x_guess = np.zeros((time_steps, usv.n_x))

    # state and input constraint
    set_state_and_input_constraints(prog, env, usv, decision_variables, time_interval, time_steps)

    # discretized dynamics
    set_dynamics(prog, usv, decision_variables, time_interval, time_steps)

    # circle obstacle constraints
    #set_circle_obstacles(prog, sphere_obstacles, decision_variables, time_steps)

    # polygon obstacle constraints
    set_polygon_obstacles(prog, env.obstacles, decision_variables, time_steps)

    # cost
    add_cost(prog, decision_variables, time_interval, time_steps)

    # solve mathematical program
    opt_start_time = default_timer()

    solver = SnoptSolver()
    result = solver.Solve(prog)

    opt_stop_time  = default_timer()


    # assert the solution
    #assert result.is_success()
    if result.is_success():

        print("Optimization Runtime:", opt_stop_time - opt_start_time)

        # retrive optimal solution
        x_opt, u_opt = [result.GetSolution(v) for v in decision_variables[:2]]

    else:

        print("No feasible solution found.")

        # Return empty vectors
        x_opt, u_opt = np.zeros((time_steps, usv.n_x)), np.zeros((time_steps, usv.n_u))

    # Stop timer
    NLP_stop_time = default_timer()

    # Print total runtime
    print("NLP Runtime:", NLP_stop_time - NLP_start_time)

    return x_opt, u_opt, x_guess
