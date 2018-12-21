using Optim
using FlightMechanics
using FlightMechanics.Simulator.Models


export steady_state_trim

mutable struct Trimmer
    ac::Aircraft
    aerostate::AeroState
    state::State
    env::Environment
    fcs::FCS
    turn_rate::Number
    gamma::Number
end


"""
    steady_state_trim(ac::Aircraft, fcs::FCS, env::Environment, tas::Number,
        pos::Position, psi::Number, gamma::Number, turn_rate::Number)

Find a steady state flight condition.

# Inputs
ac: aircraft to be trimmed.
fcs: flight control system for that aircraft. Controls given by
    `get_controls_trimmer` will be used to trim the aircraft while the rest
    will remain constant.
env: environment variables (atmospheric values, wind and gravity).
tas: true airspeed [m/s].
pos: position of the aircraft.
psi: heading of the aircraft [rad].
gamma: aerodynamic rate of climb of the aircraft [rad].
turn_rate: heading rate of change [rad/s].

# Returns
ac: trimmed aircraft.
aerostate: trimmed aerostate.
state: trimmed state.
env: environment.
fcs: trimmed FCS.

# References

See section 2.5 in [1] for the definition of steady-state flight condition.
See section 3.4 in [1] for the algorithm description.

- [1] Stevens, B. L., Lewis, F. L., (1992). Aircraft control and simulation:
 dynamics, controls design, and autonomous systems. John Wiley & Sons.
 (page 41, formula 1.4-23)
"""
function steady_state_trim(ac::Aircraft, fcs::FCS, env::Environment,
    tas::Number, pos::Position, psi::Number, gamma::Number, turn_rate::Number,
    show_trace=false)

    alpha0 = 3 * DEG2RAD
    beta0 = 0 * DEG2RAD

    phi = coordinated_turn_bank(turn_rate, alpha0, beta0, tas, gamma)
    theta = climb_theta(gamma, alpha0, beta0, phi)
    p, q, r = turn_rate_angular_velocity(turn_rate, theta, phi)

    att  = Attitude(psi, theta, phi)

    # Create initial aero and initial state
    aerostate = AeroState(tas, alpha0, beta0, get_height(pos))

    aero_vel_body = wind2body([aerostate.tas, 0, 0]..., aerostate.alpha, aerostate.beta)
    wind_vel_body = hor2body(get_wind_NED(env)..., get_euler_angles(att)...)
    vel = aero_vel_body - wind_vel_body

    state = State(
        pos,
        att,
        vel,
        [p, q, r],
        zeros(3),  # acceleration
        zeros(3)   # angular acceleration
    )

    env = calculate_environment(env, state)

    # Store every necessary variable in the trimmer
    trimmer = Trimmer(ac, aerostate, state, env, fcs, turn_rate, gamma)

    # Varibles in the trimming loop are alpha, beta, and controls.
    trim_vars0 = [alpha0, beta0] # append not fixed controls
    lower_bounds = [-15 * DEG2RAD, -15 * DEG2RAD]
    upper_bounds = [ 15 * DEG2RAD,  15 * DEG2RAD]

    for (value, range)=zip(get_controls_trimmer(fcs), get_controls_ranges_trimmer(fcs))
        min, max = range
        val = value
        append!(trim_vars0, val)
        append!(lower_bounds, min)
        append!(upper_bounds, max)
    end

    # Wrapper for trim_cost_function with trimmer
    trimming_function(x) = trim_cost_function(x, trimmer)

    # Trim
    result = optimize(trimming_function, trim_vars0,
                      #ParticleSwarm(;lower=lower_bounds, upper=upper_bounds, n_particles=250),
                      Optim.Options(
                        g_tol=1e-25,
                        iterations=1000,
                        show_trace=show_trace, show_every=50
                        );
                      )
    if show_trace
        println(result)
        println(Optim.minimizer(result))
    end

    # Return trimmed variables
    trimmer.ac, trimmer.aerostate, trimmer.state, trimmer.env, trimmer.fcs
end


function trim_cost_function(trimming_variables, trimmer::Trimmer)

    # alpha, beta, tas, height are known so aero can be created
    # alpha and beta are given by trimming_variables
    # tas is fixed for the trim
    alpha, beta = trimming_variables[1:2]

    tr = trimmer.turn_rate
    tas = get_tas(trimmer.aerostate)
    gamma = trimmer.gamma
    psi = get_euler_angles(trimmer.state)[1]

    # Impose constrains
    phi = coordinated_turn_bank(tr, alpha, beta, tas, gamma)
    theta = climb_theta(gamma, alpha, beta, phi)
    p, q, r = turn_rate_angular_velocity(tr, theta, phi)

    # Generate new state
    att = Attitude(psi, theta, phi)
    pos = get_position(trimmer.state)
    aerostate = trimmer.aerostate
    env = trimmer.env

    aero_vel_body = wind2body([aerostate.tas, 0, 0]..., aerostate.alpha, aerostate.beta)
    wind_vel_body = hor2body(get_wind_NED(env)..., get_euler_angles(att)...)
    vel = aero_vel_body - wind_vel_body

    state = State(
        pos,
        att,
        vel,
        [p, q, r],
        zeros(3),  # acceleration
        zeros(3)   # angular acceleration
    )

    aerostate = AeroState(tas, alpha, beta, get_height(state))
    env = calculate_environment(trimmer.env, trimmer.state)

    # Set trimmer attributes
    trimmer.state = state
    trimmer.aerostate = aerostate
    trimmer.env = env

    fcs = trimmer.fcs
    ac = trimmer.ac
    grav = env.grav
    # Some controls may be fixed and the rest of them are given in trimming_variables
    set_controls_trimmer(fcs, trimming_variables[3:end]...)

    trimmer.ac = calculate_aircraft(ac, fcs, aerostate, state, grav; consume_fuel=false)
    pfm = trimmer.ac.pfm
    mass_props = get_mass_props(trimmer.ac)
    mass = mass_props.mass
    inertia = mass_props.inertia

    state = get_sixdof_euler_fixed_mass_state(trimmer.state)

    r = six_dof_euler_fixed_mass(state, mass, inertia, pfm.forces, pfm.moments)[1:6]

    return sum(r.^2)
end
