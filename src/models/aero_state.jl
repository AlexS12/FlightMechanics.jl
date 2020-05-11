"""
    AeroState(alpha, beta, alpha_dot, tas, eas, cas, ias, qinf, mach)

Aerodynamic state struct storing relevant aerodynamic data.

# Constructors

Different constructors are provided:

    AeroState(alpha, beta, alpha_dot, tas, eas, cas, ias, qinf, mach)
    AeroState(state::State, env::Environment)
    AeroState(tas::Number, alpha::Number, beta::Number, height::Number)

"""
struct AeroState
    alpha :: Number  # AOA [rad]
    beta :: Number  # AOS [rad]

    alpha_dot::Number  # rad/s

    tas :: Number  # True Airspeed [m/s]
    eas :: Number  # Equivalent Airspeed [m/s]
    cas :: Number  # Calibrated Airspeed [m/s]
    ias :: Number  # Indicated Airspeed [m/s]

    qinf :: Number  # Dynamic Pressure [Pa]
    mach :: Number  # Mach number
end


function AeroState(state::State, env::Environment)

    p = get_pressure(env)
    ρ = get_density(env)
    a = get_sound_velocity(env)

    # coming from
    wind_hor = get_wind_NED(env)
    wind_body = hor2body(wind_hor..., get_euler_angles(state)...)

    aero_speed = get_body_velocity(state) + wind_body

    tas, alpha, beta = tas_alpha_beta_from_uvw(aero_speed...)

    cas = tas2cas(tas, ρ, p)
    ias = tas
    eas = tas2eas(tas, ρ)

    qinf = incompressible_qinf(tas, ρ)
    mach = tas / a

    AeroState(alpha, beta, 0, tas, eas, cas, ias, qinf, mach)
end


function AeroState(tas::Number, alpha::Number, beta::Number, p::Number, ρ::Number, a::Number)

    cas = tas2cas(tas, ρ, p)
    ias = tas
    eas = tas2eas(tas, ρ)

    qinf = incompressible_qinf(tas, ρ)
    mach = tas / a

    AeroState(alpha, beta, 0, tas, eas, cas, ias, qinf, mach)
end


show(io::IO, aerostate::AeroState) = print(
    io,
    """AeroState
    TAS=$(get_tas(aerostate)) m/s, M=$(get_mach(aerostate))
    α=$(rad2deg(get_alpha(aerostate))) deg, β=$(rad2deg(get_beta(aerostate))) deg
    qinf=$(get_qinf(aerostate)/1000.) KPa
    """
)


"""
    generate_state_aerostate(pos::Position, att::Attitude, tas::Number, alpha::Number,
                                beta::Number, env::Environment,
                                ang_vel=[0.0, 0.0, 0.0], accel=[0.0, 0.0, 0.0],
                                ang_accel=[0.0, 0.0, 0.0])

returns both state and aerostate provided the values of aircrft states, environment and
aerodynamic angles.
"""
function generate_state_aerostate(pos::Position, att::Attitude, tas::Number, alpha::Number,
                         beta::Number, env::Environment,
                         ang_vel=[0.0, 0.0, 0.0], accel=[0.0, 0.0, 0.0],
                         ang_accel=[0.0, 0.0, 0.0])

    env = calculate_environment(env, pos)

    p = get_pressure(env)
    ρ = get_density(env)
    T = get_temperature(env)
    a = get_sound_velocity(env)

    aerostate = AeroState(tas, alpha, beta, p, ρ, a)

    aero_vel_body = wind2body([tas, 0, 0]..., alpha, beta)
    wind_vel_body = hor2body(get_wind_NED(env)..., get_euler_angles(att)...)
    vel = aero_vel_body - wind_vel_body

    state = State(pos, att, vel, ang_vel, accel, ang_accel)
    return state, aerostate
end


"""
    check_state_aerostate_env_coherence(state::State, aerost::AeroState,
                                        env::Environment)

Check if aerodynamic velocity in aerostate is coherent with inertial velocity in
state and wind velocity in environment.
"""
function check_state_aerostate_env_coherence(state::State, aerost::AeroState,
                                             env::Environment)

    att = get_attitude(state)
    # wind vel from environment
    wind = get_wind_NED(env)
    # ground speed from state
    vel = get_horizon_velocity(state)
    # Composition (wind posititve blowing towards)
    aero_vel_estimation = vel + wind

    # Transform aerostate velocity to horizon
    tas, α, β = get_tas(aerost), get_alpha(aerost), get_beta(aerost)
    aero_vel = body2hor(wind2body(tas, 0, 0, α, β)..., get_euler_angles(state)...)

    isapprox(aero_vel_estimation, aero_vel)
end


get_alpha(aerost::AeroState) = aerost.alpha
get_beta(aerost::AeroState) = aerost.beta
get_aero_angles(aerost::AeroState) = [aerost.alpha, aerost.beta]
get_alpha_dot(aerost::AeroState) = aerost.alpha_dot

get_aero_speeds(aerost::AeroState) = [aerost.tas, aerost.eas, aerost.cas, aerost.ias]
get_tas(aerost::AeroState) = aerost.tas
get_eas(aerost::AeroState) = aerost.eas
get_cas(aerost::AeroState) = aerost.cas
get_ias(aerost::AeroState) = aerost.ias

get_qinf(aerost::AeroState) = aerost.qinf
get_mach(aerost::AeroState) = aerost.mach
