using FlightMechanics.Simulator.Models

export C310Engine, C310EngineLeft, C310EngineRight


abstract type C310Engine<:Engine end


struct C310EngineLeft<:C310Engine
    pfm::PointForcesMoments
    cj::Number
    power::Number
    efficiency::Number
    tanks::Array{RigidSolid, 1}
end


struct C310EngineRight<:C310Engine
    pfm::PointForcesMoments
    cj::Number
    power::Number
    efficiency::Number
    tanks::Array{RigidSolid, 1}
end


get_engine_position(prop::C310EngineLeft) = [-27.5, -70, 15.5] .* IN2M
get_engine_position(prop::C310EngineRight) = [-27.5, 70, 15.5] .* IN2M
get_engine_orientation(prop::C310Engine) = [0, 0, 0] .* DEG2RAD

# Fuel tanks
function get_default_fuel_tanks(prop::C310EngineLeft)
   [PointMass(225 * LB2KG, [35, -209.8, 28.3] .* IN2M),
    PointMass(95 * LB2KG, [35, -41.6, 11.7] .* IN2M)]
end

function get_default_fuel_tanks(prop::C310EngineRight)
   [PointMass(225 * LB2KG, [35, 209.8, 28.3] .* IN2M),
    PointMass(95 * LB2KG, [35, 41.6, 11.7] .* IN2M)]
end

function calculate_engine_power(eng::C310Engine, fcs::FCS,
                                aerost::AeroState, state::State)
   # Max power at sea level
   Pmax0 = 260 * HP2WAT
   # Power assumed proportional to thrust lever
   P0 = Pmax0 * get_thrust_setting(eng, fcs)
   # Assume supercharged engine, no variation of power with altitude until crtic altitude
   Pm = P0 * 1.0
   # Calculate fuel consumption also
   return Pm, 0
end

function calculate_propeller_thrust(eng::C310Engine, fcs::FCS,
                                    aerost::AeroState, state::State, Pm)
   # Assume constant propulsive efficiency
   ηp = 0.75
   # Calculate thrust
   thrust = ηp * Pm / aero.tas
   return thrust
end


get_thrust_setting(eng::C310EngineLeft, fcs::FCS) = get_value(fcs.t1)
get_thrust_setting(eng::C310EngineRight, fcs::FCS) = get_value(fcs.t2)
