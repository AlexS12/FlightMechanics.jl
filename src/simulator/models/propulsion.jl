using FlightMechanics


export Engine,
    get_engine_position,
    get_engine_orientation


export Propulsion,
       get_propulsion_position,
       get_fuel_mass_props,
       calculate_propulsion


abstract type Engine end

get_engine_position(eng::Engine) = [0, 0, 0]
get_engine_orientation(eng::Engine) = [0, 0, 0]
calculate_engine(eng::Engine) = error("abstract method")


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
    pos = [0.0, 0.0, 0.0]
    engines = get_engines(prop)
    for eng=engines
        pos += get_engine_position(eng)
    end
    return pos / length(engines)
end

function get_tanks(prop::Propulsion)
    tanks = RigidSolid[]
    for eng=get_engines(prop)
        for t=get_tanks(eng)
            push!(tanks, t)
        end
    end
    return tanks
end


get_fuel_mass_props(prop::Propulsion) = sum(get_tanks(prop))


function calculate_propulsion(prop::Propulsion, fcs::FCS, aerostate::AeroState,
                              state::State; consume_fuel=false)

    engines = get_engines(prop)

    power = 0.
    pfm = PointForcesMoments(get_propulsion_position(prop), [0, 0, 0], [0, 0, 0])
    cj = 0
    efficiency = 0

    for (ii, eng)=enumerate(engines)
        engines[ii] = calculate_engine(eng, fcs, aerostate, state;
                                       consume_fuel=consume_fuel)
        # TODO: use get_engine_orientation and rotate every engine pfm to body
        pfm += rotate(get_pfm(engines[ii]), get_engine_orientation(eng)...)
        power += get_power(engines[ii])
        # TODO: cj and efficiency
    end

    Propulsion(pfm, cj, power, efficiency, engines)
end
