using Test
using FlightMechanics
using FlightMechanics.Models


# TEST check_state_aerostate_env_coherence
# STATE
pos = PositionEarth(0.0, 0.0, 0.0)
att = Attitude(0.0, 0.0, 0.0)
vel = [100.0, 0.0, 0.0]
ang_vel = [0.0, 0.0, 0.0]
accel = [0.0, 0.0, 0.0]
ang_accel = [0.0, 0.0, 0.0]
state = State(pos, att, vel, ang_vel, accel, ang_accel)

# ENVIRONMENT
atm = AtmosphereISA(pos)
wind = ConstantWind(0.0, 10.0, 0.0)
grav = EarthConstantGravity()
env = Environment(atm, wind, grav)

env = calculate_environment(env, get_position(state))

# AEROSTATE
tas = 110
alpha, beta = 0.0, 0.0
p = get_pressure(env)
ρ = get_density(env)
a = get_sound_velocity(env)

aerost = AeroState(tas, alpha, beta, p, ρ, a)

@test isequal(check_state_aerostate_env_coherence(state, aerost, env), true)

# TEST generate_state_aerostate for the same case
state, aerost = generate_state_aerostate(pos, att, tas, alpha, beta, env,
                                         ang_vel, accel, ang_accel)
@test isequal(check_state_aerostate_env_coherence(state, aerost, env), true)
@test isapprox(get_horizon_velocity(state), [100, 0.0, 0.0])

# Opposite direction
# AEROSTATE
tas = 90
alpha, beta = 0.0, 0.0
aerost_exp = AeroState(tas, alpha, beta, p, ρ, a)

att = Attitude(pi, 0.0, 0.0)
# TEST generate_state_aerostate
state, aerost = generate_state_aerostate(pos, att, tas, alpha, beta, env, ang_vel, accel,
                                ang_accel)
@test isequal(check_state_aerostate_env_coherence(state, aerost, env), true)
@test isapprox(get_horizon_velocity(state), [-100, 0.0, 0.0])


# Opposite direction check for fail
# AEROSTATE
tas = 90
alpha, beta = 0.0, 0.0
aerost_exp = AeroState(tas, alpha, beta, p, ρ, a)

att = Attitude(pi, 0.0, 0.0)
# TEST generate_state_aerostate
state, aerost = generate_state_aerostate(pos, att, tas, alpha, beta, env, ang_vel, accel,
                                ang_accel)
state_wrong = State(get_position(state),
                    get_attitude(state),
                    get_body_velocity(state) .+ 0.5,
                    get_body_ang_velocity(state),
                    get_body_accel(state),
                    get_body_ang_accel(state)
                    )
@test isequal(check_state_aerostate_env_coherence(state_wrong, aerost, env),
              false)

# TODO: look for a name for this test
pos = PositionEarth(0.0, 0.0, -3000.0)
atm = AtmosphereISA(pos)
wind = ConstantWind(0.0, 0.0, 0.0)
grav = EarthConstantGravity()
env = Environment(atm, wind, grav)

tas = 100.0 # m/s
α = 4.0 * DEG2RAD
β = 0.0  # rad

att = Attitude(0.0, 4.0 * DEG2RAD, 0.0)
vel = wind2body(tas, 0, 0, α, β)
ang_vel = [0.0, 0.0, 0.0]
accel = [0.0, 0.0, 0.0]
ang_accel = [0.0, 0.0, 0.0]

env = calculate_environment(env, pos)
state1 = State(pos, att, vel, ang_vel, accel, ang_accel)
aerostate1 = AeroState(state1, env)

state2, aerostate2 = generate_state_aerostate(
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
