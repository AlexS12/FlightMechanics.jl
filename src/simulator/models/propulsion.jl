export Engine, Propulsion


abstract type Engine end


struct Propulsion
    pfm::PointForcesMoments
    cj::Number
    power::Number
    efficiency::Number
    engines::Array{Engine, 1}
end


# Getters
get_pfm(prop::Propulsion) = prop.pfm
get_cj(prop::Propulsion) = prop.cj
get_power(prop::Propulsion) = prop.power
get_efficiency(prop::Propulsion) = prop.efficiency
get_engines(prop::Propulsion) = prop.engines


function get_propulsion_position(prop::Propulsion)
    sum(get_engine_position(get_engines(prop))) / len(get_engines(prop))


function calculate(prop::Propulsion, fcs::FCS, aerostate::AeroState, state::State;
                   consume_fuel=false)

    engines = get_engines(prop)

    power = 0.
    pfm = PointForcesMoments(get_propulsion_position(prop), [0, 0, 0], [0, 0, 0])
    cj = 0
    efficiency = 0

    for (ii, eng)=engines
        engines[ii] = calculate(eng, fcs, aerostate, state; consume_fuel)
        # TODO: use get_engine_orientation and rotate every engine pfm to body
        pfm += rotate(get_pfm(engines[ii]), get_engine_orientation(eng)...)
        power += get_power(engines[ii])
        # TODO: cj and efficiency

    Propulsion(pfm, cj, power, efficiency, engines)
end
