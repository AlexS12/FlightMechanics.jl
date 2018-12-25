using FlightMechanics
using FlightMechanics.Simulator.Models
using FlightMechanics.Simulator.Aircrafts

import FlightMechanics.Simulator.Models:
    get_name, get_wing_area, get_wing_span, get_chord, get_arp,
    get_empty_mass_props


export F16,
    get_name,
    get_empty_mass_props,
    get_wing_area,
    get_wing_span,
    get_chord,
    get_arp


struct F16<:Aircraft
    mass_props::RigidSolid
    pfm::PointForcesMoments

    aerodynamics::F16Aerodynamics
    propulsion::Propulsion
end


function F16()
    pfm0 = PointForcesMoments(zeros(3), zeros(3), zeros(3))
    aero0 = F16Aerodynamics(pfm0, pfm0, pfm0, pfm0)
    engine = F16Engine()
    propulsion0 = Propulsion(
            PointForcesMoments(zeros(3), zeros(3), zeros(3)),
            0, 0, 0,
            [engine]
            )

    # mass properties cannot be retrieved until ac is created... so:
    ac = F16(RigidSolid(0, zeros(3), zeros(3, 3)),
              pfm0,
              aero0,
              propulsion0)
    mass = get_fuel_mass_props(get_propulsion(ac)) + get_empty_mass_props(ac)
            # + get_payload_mass_props(ac)
    F16(mass, pfm0, aero0, propulsion0)
end


# Name
function get_name(ac::F16)
    return "F16"
end

# GEOMETRIC PROPERTIES
get_wing_area(ac::F16) = 300.0 * FT2M^2
get_wing_span(ac::F16) = 30. * FT2M
get_chord(ac::F16) = 11.32 * FT2M
# Aerodynamic Reference Point
get_arp(ac::F16) = [0.35 * get_chord(ac), 0.0, 0.0] .* IN2M

# MASS PROPERTIES
function get_empty_mass_props(ac::F16)
    RigidSolid(
        20500 * LB2KG,                                # Empty mass
        get_arp(ac::F16),                             # Empty CG
        [9456.       0.     982.;                     # Empty inertia
            0.   55814.       0.;
          982.       0.   63100.]  .* SLUGFT2_2_KGM2
    )
end

# Crew and cargo
# get_pilot_mass_props(ac::F16) = PointMass(0 * LB2KG, [0., 0., 0.] .* IN2M)
# get_cargo_mass_props(ac::F16) = PointMass(0 * LB2KG, [0., 0., 0.] .* IN2M)

# get_payload_mass_props(ac::F16) = get_pilot_mass_props(ac)
