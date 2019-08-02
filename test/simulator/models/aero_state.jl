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

# AEROSTATE
tas = 110
alpha, beta = 0.0, 0.0
aerost = AeroState(tas, alpha, beta, 0.0)

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
aerost_exp = AeroState(tas, alpha, beta, 0.0)

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
aerost_exp = AeroState(tas, alpha, beta, 0.0)

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
