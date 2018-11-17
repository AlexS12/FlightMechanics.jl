export Aerodynamics, Aircraft


abstract type Aerodynamics end

get_pfm(aero::Aerodynamics) = aero.pfm

abstract type Aircraft end


get_mass_props(ac::Aircraft) = ac.mass_props
get_pfm(ac::Aircraft) = ac.get_mass_props
get_aerodynamics(ac::Aircraft) = ac.aerodynamics
get_propulsion(ac::Aircraft) = ac.propulsion


function calculate(ac::Aircraft, fcs::FCS, aerostate::AeroState, state::State,
                   grav::Gravity; consume_fuel=false)
    aero = get_aerodynamics(ac)
    prop = get_propulsion(ac)
    mass_props = get_mass_props(ac)

    # Calculate propulsion
    prop = calculate(prop, fcs, aerostate, state; consume_fuel)
    # Calculate aerodynamics
    aero = calculate(aero, fcs, aerostate, state)
    # Calculate gravity forces
    grav_force = calculate(grav, mass_props.mass)
    grav_pfm = PointForcesMoments(mass_props.cg, grav_force, [0, 0, 0])

    pfm = get_pfm(prop) + get_pfm(aero) + grav_pfm

    # TODO: where to discount fuel consumption?
    # Here and produce an aircraft with a mass and grav forces "not coherent"
    # or before calculating grav forces?
    # Calculate mass properties
    mass_props = (
        get_fuel_mass_props(prop) +
        get_empty_mass_props(ac) +
        get_payload_mass_props(ac)
        )
    # Return aircraft object
    typeof(ac)(mass_props, pfm, aero, prop)
end
