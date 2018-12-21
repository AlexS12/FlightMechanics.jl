using FlightMechanics.Simulator.Aircrafts
using FlightMechanics.Simulator.Models

@static if VERSION < v"0.7.0-DEV.2005"
    using Base.Test
else
    using Test
end

# Integration test for aircraft just making sure everything runs
ac = C310()

@test typeof(ac)<:Aircraft
@test get_name(ac) == "Cessna 310"

@test typeof(ac.propulsion)<:Propulsion

@test isapprox(get_wing_area(ac), 16.258032)

att = Attitude(1/180*pi, 0, 0)
pos = PositionEarth(0, 0, -1000)
state = State(pos, att, [65., 0., 3.], [0., 0., 0.], [0., 0., 0.], [0., 0., 0.])

env = DefaultEnvironment()
aerostate = AeroState(state, env)
env = calculate_environment(env, state)
grav = env.grav

fcs = C310FCS()
set_stick_lon(fcs, 0.43)
set_stick_lat(fcs, 0.562)
set_pedals(fcs, 0.5)
set_thrust(fcs, 0.68)

ac = calculate_aircraft(ac, fcs, aerostate, state, grav; consume_fuel=false)

tas = 50  # m/s
h = 300  # m
psi = pi/3  # rad
gamma = 5 * DEG2RAD
turn_rate = 0.0

ac, aerostate, state, env, fcs = steady_state_trim(ac, fcs, env, tas, pos, psi, gamma, turn_rate)
@test isapprox(ac.pfm.forces, zeros(3), atol=1e-5)
@test isapprox(ac.pfm.moments, zeros(3), atol=1e-5)
@test isapprox(get_tas(aerostate), tas, atol=1e-5)
@test isapprox(get_beta(aerostate), 0.0, atol=1e-5)
