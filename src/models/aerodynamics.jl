using FlightMechanics

export Aerodynamics,
    aerodynamics_from_wind_total, aerodynamics_from_wind_coeff,
    aerodynamics_from_body_total, aerodynamics_from_body_coeff,
    get_pfm, calculate_aerodynamics,
    get_wind_pfm, get_wind_adim_pfm,
    get_body_pfm, get_body_adim_pfm


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

function aerodynamics_from_wind_coeff(cfm::PointForcesMoments, α, β, qinf, Sw,
    b, c, aero::Aerodynamics)

    qSw = qinf * Sw

    cfm_wind = cfm  # stays the same
    pfm_wind = PointForcesMoments(cfm.point,
                                  qSw*cfm.forces,
                                  qSw*cfm.moments.*[b, c, b]
                                  )
    pfm_body = PointForcesMoments(cfm.point,
                                  wind2body(pfm_wind.forces..., α, β),
                                  wind2body(pfm_wind.moments..., α, β)
                                  )
    cfm_body = PointForcesMoments(cfm.point,
                                  pfm_body.forces/qSw,
                                  pfm_body.moments./(qSw*[b, c, b])
                                  )

    typeof(aero)(pfm_wind, cfm_wind, pfm_body, cfm_body)
end


function aerodynamics_from_body_coeff(cfm::PointForcesMoments, α, β, qinf, Sw,
    b, c, aero::Aerodynamics)

    qSw = qinf * Sw

    cfm_body = cfm  # Stays the same
    pfm_body = PointForcesMoments(cfm.point,
                                  qSw*cfm.forces,
                                  qSw*cfm.moments.*[b, c, b]
                                  )
    pfm_wind = PointForcesMoments(cfm.point,
                                  body2wind(pfm_body.forces..., α, β),
                                  body2wind(pfm_body.moments..., α, β)
                                  )
    cfm_wind = PointForcesMoments(cfm.point,
                                  pfm_wind.forces / qSw,
                                  pfm_wind.moments ./ (qSw * [b, c, b])
                                  )

    typeof(aero)(pfm_wind, cfm_wind, pfm_body, cfm_body)
end


function aerodynamics_from_wind_total(pfm::PointForcesMoments, α, β, qinf, Sw,
    b, c, aero::Aerodynamics)

    qSw = qinf * Sw

    pfm_wind = pfm  # Stays the same
    cfm_wind = PointForcesMoments(pfm.point,
                                  pfm.forces/qSw,
                                  pfm.moments./qSw./[b, c, b]
                                 )

    pfm_body = PointForcesMoments(pfm.point,
                                  wind2body(pfm.forces..., α, β),
                                  wind2body(pfm.moments..., α, β)
                                 )
    cfm_body = PointForcesMoments(pfm.point,
                                  pfm_body.forces ./ qSw,
                                  pfm_body.moments ./qSw./[b, c, b]
                                 )

    typeof(aero)(pfm_wind, cfm_wind, pfm_body, cfm_body)
end


function aerodynamics_from_body_total(pfm::PointForcesMoments, α, β, qinf, Sw,
    b, c, aero::Aerodynamics)

    qSw = qinf * Sw

    pfm_body = pfm  # Stays the same
    cfm_body = PointForcesMoments(pfm.point,
                                  pfm.forces/qSw,
                                  pfm.moments./qSw./[b, c, b]
                                 )
    pfm_wind = PointForcesMoments(pfm_body.point,
                                  body2wind(pfm_body.forces..., α, β),
                                  body2wind(pfm_body.moments..., α, β)
                                 )
    cfm_wind = PointForcesMoments(pfm_wind.point,
                                  pfm_wind.forces / qSw,
                                  pfm_wind.moments ./ (qSw * [b, c, b])
                                  )

    typeof(aero)(pfm_wind, cfm_wind, pfm_body, cfm_body)
end
