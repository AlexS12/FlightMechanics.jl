using Optim


export steady_state_trim

mutable struct Trimmer
    ac::Aircraft
    aerostate::AeroState
    state::State
    env::Environment
    ψ_dot::Number
    γ::Number
    controls::Controls
end


function Trimmer(ac, controls, env, pos, tas, ψ, γ, ψ_dot, α, β)
    # Impose constrains
    att  = trimmer_constrains_attitude(ψ, ψ_dot, α, β, tas, γ)
    ang_vel = turn_rate_angular_velocity(ψ_dot, att.theta, att.phi)

    # There is no information to set acceleration and angular acceleration at this point.
    accel = [0., 0., 0.]
    ang_accel = [0., 0., 0.]

    state, aerostate = generate_state_aerostate(
        pos,
        att,
        tas,
        α,
        β,
        env,
        ang_vel,
        accel,
        ang_accel,
    )

    return Trimmer(ac, aerostate, state, env, ψ_dot, γ, controls)
end


"""
    steady_state_trim(
        ac::Aircraft, controls::Controls, env::Environment, tas::Number, pos::Position,
        ψ::Number, γ::Number, ψ_dot::Number, α0::Number, β0::Number;
        show_trace = false, g_tol = 1e-25, max_iters = 5000
        )

Find a steady state flight condition.

Three different conditions can be sought:
- Steady level longitudinal flight: bank angle μ=0 and flight-path angle γ=0.
- Steady level turns: turn_rate≠0 and flight-path angle γ=0.
- Steady longitudinal climbs or descents: γ≠0 and bank angle μ=0.
- Steady turning climbs or descents: γ≠0 and bank angle μ≠0.

# Inputs
ac: aircraft to be trimmed.
controls : controls to be used to trim the aircraft with their initial values.
env: environment variables (atmospheric values, wind and gravity).
tas: trimming true airspeed [m/s].
pos: position of the aircraft.
ψ: heading of the aircraft [rad].
γ: aerodynamic rate of climb of the aircraft [rad].
ψ_dot: heading rate of change [rad/s].
α0, β0: Inital AOA and AOS for trimmer algorithm.
show_trace: show trimming information and algorithm trace. Optional, default=true.

# Returns
ac: trimmed aircraft.
aerostate: trimmed aerostate.
state: trimmed state.
controls: trimmed controls

# Notes

Steady flight is defined as flight where the aircraft's linear and angular velocity vectors
are constant in a body-fixed reference frame (ie. body frame or wind frame).

SixDOFEulerFixedMass is always used for state derivative calculation during trimming.

# References

See section 2.5 in [1] for the definition of steady-state flight condition.
See section 3.4 in [1] for the algorithm description.

- [1] Stevens, B. L., Lewis, F. L., (1992). Aircraft control and simulation:
 dynamics, controls design, and autonomous systems. John Wiley & Sons.
 (page 41, formula 1.4-23)
"""
function steady_state_trim(
    ac::Aircraft, controls::Controls, env::Environment, tas::Number, pos::Position,
    ψ::Number, γ::Number, ψ_dot::Number, α0::Number, β0::Number;
    show_trace = false, g_tol = 1e-25, max_iters = 5000
    )

    # Store FCS configuration to be restored at the end.
    # Allow out of range values during trimming.
    fcs = get_fcs(ac)
    allow_oor_controls = get_allow_out_of_range_inputs(fcs)
    err_on_oor_controls = get_throw_error_on_out_of_range_inputs(fcs)
    set_allow_out_of_range_inputs!(fcs, true)
    set_throw_error_on_out_of_range_inputs!(fcs, false)

    # Initialize trimmer
    trimmer = Trimmer(ac, controls, env, pos, tas, ψ, γ, ψ_dot, α0, β0)

    # variables to trim the ac -> x0 = [α, β, controls_to_be_trimmed...]
    x0 = [α0, β0, get_controls_array(controls)...]

    # Calcualte cost function to check if a/c is already trimmed
    # Calcualte aircraft
    grav = env.grav
    ac = calculate_aircraft(
        ac, controls, trimmer.aerostate, trimmer.state, grav; consume_fuel = false
        )
    trimmer.ac = ac
    cost = evaluate_cost_function(trimmer)

    # Wrapper for trim_cost_function with trimmer
    trimming_function(x) = trim_cost_function(x, trimmer)

    # Trim if not already trimmed
    # XXX: It is observed that sometimes optimization method returns a minimum slightly
    # above g_tol, so the optmization loop would be entered again in a retrimming.
    # See tests -> trimmer.jl
    if cost > g_tol * 10
        # Trim
        result = optimize(
            trimming_function,
            x0,
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

    # Reconfigure fcs to initial configuration
    fcs = get_fcs(trimmer.ac)
    set_allow_out_of_range_inputs!(fcs, allow_oor_controls)
    set_throw_error_on_out_of_range_inputs!(fcs, err_on_oor_controls)

    # Return trimmed variables
    return trimmer.ac, trimmer.aerostate, trimmer.state, trimmer.controls
end


function trimmer_constrains_attitude(ψ, ψ_dot, α, β, tas, γ)
    ϕ = coordinated_turn_bank(ψ_dot, α, β, tas, γ)
    θ = climb_theta(γ, α, β, ϕ)
    return Attitude(ψ, θ, ϕ)
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

    # Unpack x with trimming values candidates
    # α and β are given by x
    α, β = x[1:2]
    controls = typeof(trimmer.controls)(x[3:end]...)

    ψ_dot = trimmer.ψ_dot
    tas = get_tas(trimmer.aerostate)
    γ = trimmer.γ
    ψ = get_euler_angles(trimmer.state)[1]

    # Impose constrains
    att  = trimmer_constrains_attitude(ψ, ψ_dot, α, β, tas, γ)
    ang_vel = turn_rate_angular_velocity(ψ_dot, att.theta, att.phi)

    pos = get_position(trimmer.state)
    aerostate = trimmer.aerostate
    accel = [0., 0., 0.]
    ang_accel = [0., 0., 0.]

    env = trimmer.env

    state, aerostate = generate_state_aerostate(
        pos,
        att,
        tas,
        α,
        β,
        env,
        ang_vel,
        accel,
        ang_accel
    )

    env = calculate_environment(trimmer.env, get_position(trimmer.state))

    ac = trimmer.ac
    grav = env.grav
    # Calcualte aircraft
    ac = calculate_aircraft(ac, controls, aerostate, state, grav; consume_fuel = false)

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
