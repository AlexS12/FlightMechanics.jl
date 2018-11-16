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

#Getters
get_pfm(eng::C310Engine) = eng.pfm
get_cj(eng::C310Engine) = eng.cj
get_power(eng::C310Engine) = eng.power
get_efficiency(eng::C310Engine) = eng.efficiency
get_tanks(eng::C310Engine) = eng.tanks


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
                                aerostate::AeroState, state::State)
   # Max power at sea level
   Pmax0 = 260 * HP2WAT
   # Power assumed proportional to thrust lever
   P0 = Pmax0 * get_thrust_setting(eng, fcs)
   # Assume supercharged engine, no variation of power with altitude until crtic altitude
   Pm = P0 * 1.0
   # Calculate fuel consumption also
   cj = 0.0
   return Pm, cj
end

function calculate_propeller_thrust(eng::C310Engine, fcs::FCS,
                                    aerostate::AeroState, state::State, Pm)
   # Assume constant propulsive efficiency
   ηp = 0.75
   # Calculate thrust
   thrust = ηp * Pm / aero.tas
   return thrust, ηp
end


function calculate(eng::C310Engine, fcs::FCS, aerostate::AeroState, state::State;
                   consume_fuel=false)
    Pm, cj = calculate_engine_power(eng, fcs, aeroste)
    thrust, ηp = calculate_propeller_thrust(eng, fcs, aerostate, state)

    if consume_fuel==true
        error("fuel consumption not implemented")
    else
        tanks = get_tanks(eng)

    # TODO: Torque calcualtion is missing
    pfm = PointForcesMoments(get_engine_position(prop),
                             [thrust, 0, 0],
                             [0, 0, 0]
                             )

    return typeof(eng)(pfm, cj, power, ηp, tanks)


get_thrust_setting(eng::C310EngineLeft, fcs::FCS) = get_value(fcs.t1)
get_thrust_setting(eng::C310EngineRight, fcs::FCS) = get_value(fcs.t2)
