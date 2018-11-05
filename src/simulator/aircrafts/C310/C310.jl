using FlightMechanics
using FlightMechanics.Simulator.Models
using FlightMechanics.Simulator.Aircrafts


export C310,
    get_name,
    get_payload_mass_props,
    get_empty_mass_props,
    get_wing_area,
    get_wing_span,
    get_chord,
    get_arp


struct C310<:Aircraft
    mass_props::RigidSolid
    pfm::PointForcesMoments

    aerodynamics::C310Aerodynamics
    propulsion::Propulsion
    fcs::FCS
end


function C310()
    mass_props0 = RigidSolid(0, zeros(3), zeros(3, 3))
    pfm0 = PointForcesMoments(zeros(3), zeros(3), zeros(3))

    aero_state0 = AeroState(zeros(9)...)
    aero0 = C310Aerodynamics(pfm0, aero_state0)

    engine_right = C310EngineRight(pfm0, 0, 0, 0,
                                   [PointMass(225 * LB2KG, [35, 209.8, 28.3] .* IN2M),
                                    PointMass(95 * LB2KG, [35, 41.6, 11.7] .* IN2M)
                                    ]
                                   )
    engine_left = C310EngineLeft(pfm0, 0, 0, 0,
                                 [PointMass(225 * LB2KG, [35, -209.8, 28.3] .* IN2M),
                                  PointMass(95 * LB2KG, [35, -41.6, 11.7] .* IN2M)
                                 ]
                                )
    propulsion0 = Propulsion(pfm0, 0, 0, 0, [engine_right, engine_left])

    fcs0 = C310FCS()

    C310(mass_props0, pfm0, aero0, propulsion0, fcs0)
end



# Name
function get_name(ac::C310)
    return "Cessna 310"
end


# GEOMETRIC PROPERTIES
get_wing_area(ac::C310) = 175.0 * FT2M^2
get_wing_span(ac::C310) = 36.5 * FT2M
get_chord(ac::C310) = 4.9 * FT2M
# Aerodynamic Reference Point
get_arp(ac::C310) = [46, 0, 8.6] .* IN2M

# MASS PROPERTIES
function get_empty_mass_props(ac::C310)
    RigidSolid(
        2950 * LB2KG,                            # Empty mass
        [46, 0, 8.6] .* IN2M,                    # Empty CG
        [8884      0      0;                     # Empty inertia
            0   1939      0;
            0      0  11001]  .* SLUGFT2_2_KGM2
    )
end

# Crew and cargo
get_pilot_mass_props(ac::C310) = PointMass(180 * LB2KG, [37, -14, 24] .* IN2M)
get_copilot_mass_props(ac::C310) = PointMass(180 * LB2KG, [37, 14, 24] .* IN2M)
get_lugage_mass_props(ac::C310) = PointMass(100 * LB2KG, [90, -0, 24] .* IN2M)

get_payload_mass_props(ac::Aircraft) = (get_pilot_mass_props(ac) +
                                        get_copilot_mass_props(ac) +
                                        get_lugage_mass_props(ac)
                                        )
