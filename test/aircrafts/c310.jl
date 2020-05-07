using Test
using FlightMechanics
using FlightMechanics.Models
using FlightMechanics.Aircrafts


# Integration test for aircraft just making sure everything runs
ac = C310()

@test typeof(ac)<:Aircraft
@test get_name(ac) == "Cessna 310"

@test typeof(ac.propulsion)<:Propulsion

@test isapprox(get_wing_area(ac), 16.258032)

att = Attitude(1/180*pi, 0, 0)
pos = EarthPosition(0, 0, -1000)
state = State(pos, att, [65., 0., 3.], [0., 0., 0.], [0., 0., 0.], [0., 0., 0.])

env = Environment(pos, atmos="ISA1978", wind="NoWind", grav="const")
aerostate = AeroState(state, env)

controls = StickPedalsLeverControls(0.43, 0.562, 0.5, 0.68)

grav = get_gravity(env)
ac = calculate_aircraft(ac, controls, aerostate, state, grav; consume_fuel=false)

tas = 50  # m/s
h = 300  # m
psi = pi/3  # rad
gamma = 5 * DEG2RAD
turn_rate = 0.0

ac, aerostate, state, fcs = steady_state_trim(
    ac, controls, env, tas, pos, psi, gamma, turn_rate, 5.0*DEG2RAD, 0.0*DEG2RAD
    )

@test isapprox(ac.pfm.forces, zeros(3), atol=1e-5)
@test isapprox(ac.pfm.moments, zeros(3), atol=1e-5)
@test isapprox(get_tas(aerostate), tas, atol=1e-5)
@test isapprox(get_beta(aerostate), 0.0, atol=1e-5)
