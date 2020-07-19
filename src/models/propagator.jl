"""
    propagate_timestep(
        ac, controls, env, state, aerostate, tspan, dynamic_system, solver; 
        solver_options=Dict(),
    )

Propagate tspan time step considering that the aircraft and the controls are constant.

# Arguments
`ac::Aircraft`: aircraft model. Includes engine, aerodynamics, flight control system...
`controls::Controls`: controls that will be set on aircraft fot the timestep.
`env::Environment`: environment model. Includes atmosphere, wind and gravity.
`aerostate::AeroState`: aerodynamic state.
`tspan`: tuple with tini and tfin (s).
`dynamic_system::DynamicSystem`: dynamic system model equations to be integrated.
`solver`: ODE solver from `OrdinaryDiffEq`.
`solver_options`: options to be passed to the solver.
"""
function propagate_timestep(
    ac, controls, env, state, aerostate, tspan, dynamic_system, solver;
    solver_options=Dict(),
    )

    # Controls, Forces, moments, mass, inertia and gyro effects are assumed
    # to be constant for the time step
    ac = calculate_aircraft(ac, controls, aerostate, state, get_gravity(env))

    # Unpack properties to match OrdinaryDiffEq interface
    ac_mass_props = get_mass_props(ac)
    ac_pfm = get_pfm(ac)
    h = get_gyro_effects(get_propulsion(ac))

    mass = get_mass(ac_mass_props)
    inertia = get_inertia(ac_mass_props)
    forces = get_forces(ac_pfm)
    moments = get_moments(ac_pfm)

    p = [mass, inertia, forces, moments, h]

    x0 = get_x(dynamic_system, state)

    # Create ODE problem
    prob = ODEProblem(
        get_state_equation_ode_wrapper(dynamic_system),
        x0,
        tspan,
        p,
    )
    sol = solve(prob, solver, save_everystep = false, save_start = false; solver_options...)

    if ~(sol.retcode == :Success)
        error("Propagate time step did not converge from $(tspan[1]) to $(tspan[2])")
    end

    dynamic_system.x[:] = sol.u[1]
    # TODO: is it possible to obtain this from solve?
    dynamic_system.x_dot[:] = get_state_equation(dynamic_system)(sol.u[1], p...)

    # Obtain state from sol
    state = convert(state, dynamic_system)

    # As the position has changed, environment changes (must be updated)
    # and aircraft changes (must be updated)
    # Calculate environment and aerostate for the current state
    env = calculate_environment(env, get_position(state))
    aerostate = AeroState(state, env)
    # Calculate aircraft
    ac = calculate_aircraft(ac, controls, aerostate, state, get_gravity(env))

    return sol, ac, env, state, aerostate
end


"""
    propagate(
        ac, env, state, controls_stream, tini, tfin, dt, dynamic_system, solver;
        solver_options=Dict()
    )

Integrate case to obtain aircraft trajectory by calling the solver for each time step.
Returns a ResultsStore object.

# Arguments
`ac::Aircraft`: aircraft model. Includes engine, aerodynamics, flight control system...
`env::Environment`: environment model. Includes atmosphere, wind and gravity.
`controls_stream::ControlsStream`: defines a getter for each control at time `t`.
`tini`: initial time (s).
`tfin`: final time (s).
`dt`: time step (s).
`dynamic_system::DynamicSystem`: dynamic system model equations to be integrated.
`solver`: ODE solver from `OrdinaryDiffEq`.
`solver_options`: options to be passed to the solver.
"""
function propagate(
    ac, env, state, aerostate, controls_stream, tini, tfin, dt, dynamic_system, solver;
    solver_options=Dict()
    )

    # Prepare initial time step
    t0 = tini
    t1 = t0 + dt

    # Create results object
    results = ResultsStore(tini, ac, env, state, aerostate)

    while t1 < tfin + 0.5*dt
        # Get controls
        controls = get_controls(controls_stream, t0)

        tspan = (t0, t1)
        sol, ac, env, state, aerostate = propagate_timestep(
            ac,
            controls,
            env,
            state,
            aerostate,
            tspan,
            dynamic_system,
            solver;
            solver_options=solver_options,
        )

        # Store
        push!(results, t1, ac, env, state, aerostate)

        # Prepare for next time step
        t0 = t1
        t1 = t1 + dt
    end

    return results
end