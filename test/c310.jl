using FlightMechanics.Simulator.Aircrafts
using FlightMechanics.Simulator.Models

@static if VERSION < v"0.7.0-DEV.2005"
    using Base.Test
else
    using Test
end

env = DefaultEnvironment()


ac = C310()
fcs = C310FCS()

@test typeof(ac)<:Aircraft
@test get_name(ac) == "Cessna 310"

@test typeof(ac.propulsion)<:Propulsion

# GEOMETRIC PROPERTIES
println(get_wing_area(ac))
println(get_wing_span(ac))
println(get_chord(ac))
println(get_arp(ac))

# MASS PROPERTIES
println(get_empty_mass_props(ac))
println(get_payload_mass_props(ac))
# println(get_fuel_mass_props(ac))
println(ac.mass_props)

att = Attitude(1/180*pi, 0, 0)
state = EarthBodyState(0, 0, -1000, 65, 0, 3, 0, 0, 0, att, 0, 0, 0, 0, 0, 0)
aerostate = AeroState(state, env)
env = calculate_environment(env, state)
grav = env.grav

println(grav)

set_stick_lon(fcs, 0.43)
set_stick_lat(fcs, 0.562)
set_pedals(fcs, 0.5)
set_thrust(fcs, 0.68)
ac = calculate_aircraft(ac, fcs, aerostate, state, grav; consume_fuel=false)
println(ac.pfm)

tas = 50  # m/s
h = 300  # m
psi = pi/3  # rad
gamma = 11 * DEG2RAD
turn_rate = 0.0

trimmer = steady_state_trim(ac, fcs, env, tas, h, psi, gamma, turn_rate)
println(trimmer.ac.pfm)
