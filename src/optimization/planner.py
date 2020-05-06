import numpy as np

from .a_star import AStar
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

    # Slack variable
    s = prog.NewContinousVariables(rows=time_steps + 1, cols=N_obst, name='s')

    return x, u, lambdas, s


# ----------------------------------------------------
# Set initial and final condition for the
# optimization problem
# ----------------------------------------------------
def set_initial_and_terminal_position(prog, start, goal, decision_variables):

    # Unpack state and input
    x, u = decision_variables[:2]
    #pdb.set_trace()

    # Initial state
    x_0 = np.concatenate((start, np.array([0, 0, 0, 0])))

    # Final state
    x_N = np.concatenate((goal, np.array([0, 0, 0, 0])))

    # Enforce initial constraint
    prog.AddLinearConstraint(eq(x[0], x_0))

    # Enforce final constraint
    prog.AddLinearConstraint(eq(x[-1], x_N))


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
    astar = AStar(start, goal, polygon_obstacles, lb, ub, resolution=10)
    path  = astar.plan()

    # Calculate Dubins Segments
    d_segments, L = dubins_path(path, turning_radius=1, type='adaptive')

    # Generate initial guess in [x, y, psi]
    x_guess = dubins_generate_initial_guess(d_segments, L, time_steps)

    # Set initial guess
    #prog.SetInitialGuess(x[:,:2], p_guess[:,:2])
    prog.SetInitialGuess(x[:,:3], x_guess)


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
def set_polygon_obstacles(prog, obstacles, decision_variables, time_steps):

    # Unpack state and input
    x, u, lambdas, s = decision_variables

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
            prog.AddConstraint( (residual >= 0.01) )

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

    # initialize optimization
    prog = MathematicalProgram()

    # optimization variables
    decision_variables = add_decision_variables(prog, env.obstacles, usv.n_x, usv.n_u, time_steps)

    # intial and terminal constraint
    set_initial_and_terminal_position(prog, start, goal, decision_variables)

    # initial guess
    set_initial_guess(prog, env, start, goal, decision_variables, time_interval, time_steps)

    # discretized dynamics
    set_dynamics(prog, usv, decision_variables, time_interval, time_steps)

    # circle obstacle constraints
    #set_circle_obstacles(prog, sphere_obstacles, decision_variables, time_steps)

    # polygon obstacle constraints
    set_polygon_obstacles(prog, env.obstacles, decision_variables, time_steps)

    # cost
    add_cost(prog, decision_variables, time_interval, time_steps)


    # solve mathematical program
    solver = SnoptSolver()
    result = solver.Solve(prog)

    # assert the solution
    assert result.is_success()

    # retrive optimal solution
    x_opt, u_opt = [result.GetSolution(v) for v in decision_variables[:2]]


    return x_opt, u_opt











# ----------------------------------------------------
# Set circular obstacles as a constraint for the
# optimization problem TODO remove if still unused
# ----------------------------------------------------
def set_circle_obstacles(prog, obstacles, decision_variables, time_steps):

    # Unpack state and input
    x, u = decision_variables[:2]

    # Enforce circular constraints dist(circle)**2 >= r**2
    for t in range(time_steps):
        for circle in obstacles:
            p = x[t,:2] - circle.position
            residual = p.dot(p) - circle.radius**2
            prog.AddConstraint(residual >= 0)


# ----------------------------------------------------
# TODO remove, or modify severily..
# ----------------------------------------------------
def set_polygonial_obstacles(prog, obstacles, decision_variables, start, goal, time_steps, n_po):

    # Unpack state, input and binary
    x, u, delta = decision_variables[:3]

    # big-M vector
    M = get_big_M(start, goal)

    # Enforce polygon constraints Ax >= b
    for t in range(time_steps) :
        for n in range(n_po):
            polygon = obstacles[n]

            A = polygon.A
            b = polygon.b

            n_sides = len(b)

            M_n = np.array(M * n_sides)

            prog.AddLinearConstraint(
                ge(polygon.A.dot(x[t,:2]), polygon.b - (1 - delta[t, n])*M_n)
            )


# ----------------------------------------------------
# TODO remove
# ----------------------------------------------------
def set_safe_regions(prog, env, decision_variables, start, goal, time_steps):

    # Unpack state, input and binary
    x, u, delta = decision_variables[:3]

    # big-M vector
    M = get_big_M(start, goal)

    # Number of safe_regions
    n_sr = len(env.safe_regions)

    # Enforce polygon constraints Ax >= b
    for t in range(time_steps) :
        for n in range(n_sr):
            A, b, v = env.safe_regions[n]

            A = np.array(A)
            b = np.array(b)

            n_sides = len(b)

            M_n = np.array(M * n_sides)

            prog.AddLinearConstraint(
                le(A.dot(x[t,:2]), b + (1 - delta[t, n])*M_n)
            )



# ----------------------------------------------------
# TODO remove
# ----------------------------------------------------
def set_binary(prog, decision_variables, time_steps):

    # Unpack state, input and binary
    x, u, delta = decision_variables[:3]

    # Enforce binary constraint
    for t in range(time_steps + 1):
        sum_delta = delta[t].sum()
        prog.AddLinearConstraint(sum_delta == 1)



















# TODO REMOVE BELOW -------------------------------------------------------------------
def get_big_M(start, goal):
    M = []

    p = goal - start

    dist = np.sqrt(p.dot(p))
    M.append(dist)

    return M






# function that interpolates two given positions of the rocket
# velocity is set to zero for all the times
def interpolate_rocket_state(p_initial, p_final, time_interval=0.2, time_steps=100):

    # initial and final time and state
    time_limits = [0., time_steps * time_interval]
    position_limits = np.column_stack((p_initial, p_final))
    state_limits = np.vstack((position_limits, np.zeros((2, 2))))

    # linear interpolation in state
    state = PiecewisePolynomial.FirstOrderHold(time_limits, state_limits)

    # sample state on the time grid
    state_guess = np.vstack([state.value(t * time_interval).T for t in range(time_steps + 1)])

    return state_guess
