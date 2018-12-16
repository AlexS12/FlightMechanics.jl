export Aircraft,
    get_mass_props, get_pfm, get_aerodynamics, get_propulsion,
    get_name, get_wing_area, get_wing_span, get_chord, get_arp,
    get_empty_mass_props, get_payload_mass_props,
    calculate_aircraft


abstract type Aircraft end


get_mass_props(ac::Aircraft) = ac.mass_props
get_pfm(ac::Aircraft) = ac.get_mass_props
get_aerodynamics(ac::Aircraft) = ac.aerodynamics
get_propulsion(ac::Aircraft) = ac.propulsion

get_name(ac::Aircraft) = "Generic Aricraft"
get_wing_area(ac::Aircraft) = 0.0  # m²
get_wing_span(ac::Aircraft) = 0.0  # m
get_chord(ac::Aircraft) = 0.0  # m
get_arp(ac::Aircraft) = [0, 0, 0]  # m

get_empty_mass_props(ac::Aircraft) = RigidSolid(0.0, zeros(3), zeros(3, 3))
get_payload_mass_props(ac::Aircraft) = RigidSolid(0.0, zeros(3), zeros(3, 3))


function calculate_aircraft(ac::Aircraft, fcs::FCS, aerostate::AeroState,
                            state::State, grav::Gravity; consume_fuel=false)
    aero = get_aerodynamics(ac)
    prop = get_propulsion(ac)
    mass_props = get_mass_props(ac)

    # Calculate propulsion
    prop = calculate_propulsion(prop, fcs, aerostate, state; consume_fuel=consume_fuel)
    # Calculate aerodynamics
    aero = calculate_aerodynamics(ac, aero, fcs, aerostate, state)
    # Calculate gravity forces
    grav_force = get_grav_body_vector(grav) * mass_props.mass
    grav_pfm = PointForcesMoments(mass_props.cg, grav_force, [0, 0, 0])

    β = get_beta(aerostate)
    α = get_alpha(aerostate)
    pfm = grav_pfm + get_pfm(prop) + get_pfm(aero)

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
