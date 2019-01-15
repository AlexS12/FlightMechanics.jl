using Test
using FlightMechanics
using FlightMechanics.Simulator.Models


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

# TEST state_aerostate for the same case
state, aerost = state_aerostate(pos, att, tas, alpha, beta, env, ang_vel, accel,
                                ang_accel)
@test isequal(check_state_aerostate_env_coherence(state, aerost, env), true)
@test isapprox(get_horizon_velocity(state), [100, 0.0, 0.0])

# Opposite direction
# AEROSTATE
tas = 90
alpha, beta = 0.0, 0.0
aerost_exp = AeroState(tas, alpha, beta, 0.0)

att = Attitude(pi, 0.0, 0.0)
# TEST state_aerostate
state, aerost = state_aerostate(pos, att, tas, alpha, beta, env, ang_vel, accel,
                                ang_accel)
@test isequal(check_state_aerostate_env_coherence(state, aerost, env), true)

@test isapprox(get_horizon_velocity(state), [-100, 0.0, 0.0])
