using FlightMechanics

export Aerodynamics,
    get_pfm, calculate_aerodynamics


"""
    Aerodynamics

Aerodynamic forces and moments information
"""
abstract type Aerodynamics end

get_wind_pfm(aero::Aerodynamics) = aero.wind_pfm
get_wind_adim_pfm(aero::Aerodynamics) = aero.wind_coeff_pfm

get_body_pfm(aero::Aerodynamics) = aero.body_pfm
get_body_adim_pfm(aero::Aerodynamics) = aero.body_coeff_pfm

get_pfm(aero::Aerodynamics) = get_body_pfm(aero)

get_cL(aero::Aerodynamics) = -aero.wind_coeff_pfm.forces[3]
get_cY(aero::Aerodynamics) = aero.wind_coeff_pfm.forces[2]
get_cD(aero::Aerodynamics) = -aero.wind_coeff_pfm.forces[1]

get_clong(aero::Aerodynamics) = aero.wind_coeff_pfm.forces[3]
get_clat(aero::Aerodynamics) = aero.wind_coeff_pfm.forces[2]
get_cnorm(aero::Aerodynamics) = -aero.wind_coeff_pfm.forces[1]

function calculate_aerodynamics(aero::Aerodynamics, fcs::FCS,
    aerostate::AeroState, state::State)
    error("abstract method")
end
