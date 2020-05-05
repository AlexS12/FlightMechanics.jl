using Optim


export steady_state_trim

mutable struct Trimmer
    ac::Aircraft
    aerostate::AeroState
    state::State
    env::Environment
    turn_rate::Number
    gamma::Number
    controls::Controls
end


"""
    steady_state_trim(ac::Aircraft, env::Environment, tas::Number,
        pos::Position, psi::Number, gamma::Number, turn_rate::Number,
        α0::Number, β0::Number; show_trace=false)

Find a steady state flight condition. Steady flight is defined as flight where
the aircraft's linear and angular velocity vectors are constant in a body-fixed
reference frame (ie. body frame or wind frame).

Three different conditions can be sought:
- Steady level longitudinal flight: bank angle μ=0 and flight-path angle γ=0.
- Steady level turns: turn_rate≠0 and flight-path angle γ=0.
- Steady longitudinal climbs or descents: γ≠0 and bank angle μ=0.
- Steady turning climbs or descents: γ≠0 and bank angle μ≠0.

# Inputs
ac: aircraft to be trimmed.
fcs: flight control system. Controls given by `get_controls_trimmer` will be
  used to trim the aircraft while the rest will remain constant.
env: environment variables (atmospheric values, wind and gravity).
tas: true airspeed [m/s].
pos: position of the aircraft.
psi: heading of the aircraft [rad].
gamma: aerodynamic rate of climb of the aircraft [rad].
turn_rate: heading rate of change [rad/s].
α0, β0: Inital AOA and AOS for trimmer algorithm.
show_trace: show trimming information and algorithm trace. Optional, default=true.

# Returns
ac: trimmed aircraft.
aerostate: trimmed aerostate.
state: trimmed state.
controls: trimmed controls

# References

See section 2.5 in [1] for the definition of steady-state flight condition.
See section 3.4 in [1] for the algorithm description.

- [1] Stevens, B. L., Lewis, F. L., (1992). Aircraft control and simulation:
 dynamics, controls design, and autonomous systems. John Wiley & Sons.
 (page 41, formula 1.4-23)
"""
function steady_state_trim(ac::Aircraft, controls::Controls, env::Environment,
    tas::Number, pos::Position, psi::Number, gamma::Number, turn_rate::Number,
    α0::Number, β0::Number; show_trace = false, g_tol = 1e-25, max_iters = 5000)

    alpha0 = α0
    beta0 = β0

    phi = coordinated_turn_bank(turn_rate, alpha0, beta0, tas, gamma)
    theta = climb_theta(gamma, alpha0, beta0, phi)
    att  = Attitude(psi, theta, phi)

    p, q, r = turn_rate_angular_velocity(turn_rate, theta, phi)
    ang_vel = [p, q, r]

    accel = [0., 0., 0.]
    ang_accel = [0., 0., 0.]

    state, aerostate = generate_state_aerostate(
        pos,
        att,
        tas,
        alpha0,
        beta0,
        env,
        ang_vel,
        accel,
        ang_accel,
    )

    # Ensure that environment is calculated at the position given to the trimmer
    env = calculate_environment(env, get_position(state))
    set_controls!(ac, controls, allow_out_of_range=true)

    # Store every necessary variable in the trimmer
    trimmer = Trimmer(ac, aerostate, state, env, turn_rate, gamma, controls)

    # # Varibles in the trimming loop are alpha, beta, and controls.
    trim_vars0 = [alpha0, beta0] # append not fixed controls
    # lower_bounds = [-15 * DEG2RAD, -15 * DEG2RAD]
    # upper_bounds = [ 15 * DEG2RAD,  15 * DEG2RAD]
    
    for value in get_controls_array(controls)
        append!(trim_vars0, value)
        # append!(lower_bounds, min)
        # append!(upper_bounds, max)
    end

    # Wrapper for trim_cost_function with trimmer
    trimming_function(x) = trim_cost_function(x, trimmer)

    # Calcualte cost function to check if a/c is already trimmed
    # Calcualte aircraft
    grav = env.grav
    ac = calculate_aircraft(ac, aerostate, state, grav; consume_fuel = false)
    trimmer.ac = ac
    cost = evaluate_cost_function(trimmer)

    # Trim if not already trimmed
    # XXX: It is observed that sometimes optimization method returns a minimum slightly
    # above g_tol, so the optmization loop would be entered again in a retrimming.
    # See tests -> trimmer.jl
    if cost > g_tol*10
        # Trim
        result = optimize(
            trimming_function,
            trim_vars0,
            Optim.Options(
                g_tol = g_tol,
                iterations = max_iters,
                show_trace = show_trace,
                show_every = 100,
                );
            )

        if show_trace
            println(result)
            println(Optim.minimizer(result))
        end
    end

    # Return trimmed variables
    return trimmer.ac, trimmer.aerostate, trimmer.state, trimmer.controls
end

"""
    trim_cost_function(x, trimmer::Trimmer)

Function minimized to trim the aircraft. Independent variables are passed in x:
- x[1] angle of attack (rad).
- x[2] angle of sideslip (rad).
- x[3:end]: controls expected by set_controls_trimmer! (ie. stick_long, stick_lat, pedals,
throttle)

"""
function trim_cost_function(x, trimmer::Trimmer)

    # alpha, beta, tas, height are known so aero can be created
    # alpha and beta are given by x
    # tas is fixed for the trim
    alpha, beta = x[1:2]

    controls = typeof(trimmer.controls)(x[3:end]...)

    tr = trimmer.turn_rate
    tas = get_tas(trimmer.aerostate)
    gamma = trimmer.gamma
    psi = get_euler_angles(trimmer.state)[1]

    # Impose constrains
    phi = coordinated_turn_bank(tr, alpha, beta, tas, gamma)
    theta = climb_theta(gamma, alpha, beta, phi)
    att = Attitude(psi, theta, phi)
    p, q, r = turn_rate_angular_velocity(tr, theta, phi)
    ang_vel = [p, q, r]
    pos = get_position(trimmer.state)
    aerostate = trimmer.aerostate
    accel = [0., 0., 0.]
    ang_accel = [0., 0., 0.]

    env = trimmer.env

    state, aerostate = generate_state_aerostate(
        pos,
        att,
        tas,
        alpha,
        beta,
        env,
        ang_vel,
        accel,
        ang_accel
    )

    env = calculate_environment(trimmer.env, get_position(trimmer.state))

    ac = trimmer.ac
    grav = env.grav
    # Some controls may be fixed and the rest of them are given in x
    set_controls!(ac, controls, allow_out_of_range=true)
    # Calcualte aircraft
    ac = calculate_aircraft(ac, aerostate, state, grav; consume_fuel = false)

    # Update trimmer
    trimmer.ac = ac
    trimmer.aerostate = aerostate
    trimmer.state = state
    trimmer.env = env
    trimmer.controls = controls
    
    # Calculate objective function cost
    cost = evaluate_cost_function(trimmer)
    return cost
end


function evaluate_cost_function(trimmer::Trimmer)
    # Calculate derivatives u_dot, v_dot, w_dot, p_dot, q_dot, r_dot
    pfm = trimmer.ac.pfm
    mass_props = get_mass_props(trimmer.ac)
    mass = mass_props.mass
    inertia = mass_props.inertia

    # Get dynamic system state x
    six_dof_euler_fixed_mass_ds = convert(SixDOFEulerFixedMass, trimmer.state)
    x = get_x(six_dof_euler_fixed_mass_ds)
    f = get_state_equation(six_dof_euler_fixed_mass_ds)
    # Evaluate x_dot given x, u, parameters
    x_dot = f(x, mass, inertia, pfm.forces, pfm.moments)[1:6]

    return sum(x_dot.^2)
end
