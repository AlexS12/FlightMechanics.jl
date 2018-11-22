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


function steady_state_trim(ac::Aircraft, fcs::FCS, env::Environment,
    tas::Number, height::Number, psi::Number, gamma::Number, turn_rate::Number)

    alpha0 = 3 * DEG2RAD
    beta0 = 0 * DEG2RAD

    phi = coordinated_turn_bank(turn_rate, alpha0, beta0, tas, gamma)
    theta = climb_theta(gamma, alpha0, beta0, phi)
    p, q, r = turn_rate_angular_velocity(turn_rate, theta, phi)

    att  = Attitude(psi, theta, phi)

    # Create initial aero and initial state
    aerostate = AeroState(tas, alpha0, beta0, height)

    # TODO: take into account wind in inertial velocity
    state = EarthBodyState(
        0, 0, -height,
        wind2body([aerostate.tas, 0, 0]..., aerostate.alpha, aerostate.beta)...,
        p, q, r,
        att,
        zeros(6)...
    )

    env = calculate_environment(env, state)

    # Store every necessary variable in the trimmer
    trimmer = Trimmer(ac, aerostate, state, env, fcs, turn_rate, gamma)

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
                        show_trace=true, show_every=25
                        );
                      )

    println(result)
    println(Optim.minimizer(result))

    # Return trimmed variables
    trimmer
end


function trim_cost_function(trimming_variables, trimmer::Trimmer)

    # alpha, beta, tas, height are known so aero can be created
    # alpha and beta are given by trimming_variables
    # tas is fixed for the trim
    alpha, beta = trimming_variables[1:2]

    tr = trimmer.turn_rate
    tas = trimmer.aerostate.tas
    gamma = trimmer.gamma
    psi = trimmer.state.att.psi

    # Impose constrains
    phi = coordinated_turn_bank(tr, alpha, beta, tas, gamma)
    theta = climb_theta(gamma, alpha, beta, phi)
    p, q, r = turn_rate_angular_velocity(tr, theta, phi)

    # Generate new state
    att  = Attitude(psi, theta, phi)
    # TODO: take into account wind in inertial velocity
    state = EarthBodyState(
        get_earth_position(trimmer.state)...,
        wind2body([tas, 0, 0]..., alpha, beta)...,
        p, q, r,
        att,
        zeros(6)...
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


function coordinated_turn_bank(turn_rate, alpha, beta, tas, gamma)

    g0 = GRAVITY_ACCEL
    G = turn_rate * tas / g0

    if abs(gamma) < 1e-8
        phi = G * cos(beta) / (cos(alpha) - G * sin(alpha) * sin(beta))
        phi = atan(phi)
    else
        a = 1 - G * tan(alpha) * sin(beta)
        b = sin(gamma) / cos(beta)
        c = 1 + G^2 * cos(beta)^2

        sq = sqrt(c * (1 - b^2) + G^2 * sin(beta)^2)

        num = (a - b^2) + b * tan(alpha) * sq
        den = a ^ 2 - b^2 * (1 + c * tan(alpha)^2)

        phi = atan(G * cos(beta) / cos(alpha) * num / den)
    end
    return phi
end


function climb_theta(gamma, alpha, beta, phi)
    a = cos(alpha) * cos(beta)
    b = sin(phi) * sin(beta) + cos(phi) * sin(alpha) * cos(beta)
    sq = sqrt(a^2 - sin(gamma)^2 + b^2)
    theta = (a * b + sin(gamma) * sq) / (a^2 - sin(gamma)^2)
    theta = atan(theta)
    return theta
end


function turn_rate_angular_velocity(turn_rate, theta, phi)
    # w = turn_rate * k_h
    # k_h = sin(theta) i_b + sin(phi) * cos(theta) j_b + cos(theta) * sin(phi)
    # w = p * i_b + q * j_b + r * k_b
    p = - turn_rate * sin(theta)
    q = turn_rate * sin(phi) * cos(theta)
    r = turn_rate * cos(theta) * sin(phi)
    return [p, q, r]
end
