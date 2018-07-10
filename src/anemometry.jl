import .AirConstants: GAMMA_AIR, P0, RHO0


γ = GAMMA_AIR
p0 = P0
rho0 = RHO0


"""
    qc2cas(qc::Real)

Calculate calibrated airspeed from ASI (Air Speed indicator), differential
pressure between impact pressure and static pressure.
qc = p_t - p_s

Implementation from:

.. [1] Ward, D. T. (1993). Introduction to flight test engineering. Elsevier
    Science Ltd. (page 13, formula 2.13)
"""
function qc2cas(qc::Real)
    cas = sqrt((2*γ*p0) / ((γ-1) * rho0) * ((qc/p0 + 1) ^(γ / (γ-1)) - 1))
    return cas
end


"""
    qc2cas(qc::Real)

Calculate true airspeed from ASI (Air Speed indicator), differential
pressure between impact pressure and static pressure (qc = p_t - p_s), rho
and p.

Implementation from:

.. [1] Ward, D. T. (1993). Introduction to flight test engineering. Elsevier
    Science Ltd. (page 12, based on formula 2.11)
"""
function qc2tas(qc::Real, rho::Real, p::Real)
    tas2 = (2*γ*p) / ((γ-1) * rho0) * ((qc/p + 1) ^(γ / (γ-1)) - 1) * rho0/rho
    tas = sqrt(tas2)
    return tas
end

"""
    qc2eas(qc::Real, p::Real)

Calculate equivalent airspeed from ASI (Air Speed indicator), differential
pressure between impact pressure and static pressure (qc = p_t - p_s) and p.

Implementation from:

.. [1] Ward, D. T. (1993). Introduction to flight test engineering. Elsevier
    Science Ltd.
"""
function qc2eas(qc::Real, p::Real)
    eas = sqrt((2*γ*p) / ((γ-1) * rho0) * ((qc/p + 1) ^(γ / (γ-1)) - 1))
    return eas
end


function tas2eas(tas, rho)
    eas = tas * sqrt(rho / rho0)
    return eas
end


function eas2tas(eas, rho)
    tas = eas / sqrt(rho / rho0)
    return tas
end


function cas2eas(cas, rho, p)
    
    return eas
end


function eas2cas(eas, rho, p)

    return cas
end


function cas2tas(cas, rho, p)

    return tas
end


function tas2cas(tas, rho, p)

    return cas
end


function tas_alpha_beta_from_uvw(u, v, w)
    
    tas = sqrt(u*u + v*v + w*w)
    alpha = atan(w / u)
    beta = asin(v / TAS)
    return tas, alpha, beta
end


function incompressible_qinf(tas, rho)
    return 0.5 * rho * tas*tas
end


function stagnation_pressure(tas, p, a)

    M = tas / a

    if M < 1
        ps = p * (1 + (γ - 1) / 2 * M^2) ^ (γ / (γ - 1))
    else
        
        ps = ((γ + 1)^2 * M^2) / (4 * γ * M^2 - 2 * (γ - 1))
        ps = ps ^ (γ / (γ - 1))
        ps = ps * (1 - γ + 2 * γ * M^2) / (γ + 1) * p
    end
    return ps
end


function sutherland_visconsity(temp)
    # TODO: move this to constants
    visc_0 = 1.176e-5  # kg(m/s)
    T0 = 273.1  # K
    b = 0.4042  # non-dimensional
    return visc_0 * (temp / T0)^(3 / 2) * ((1 + b)/((temp / T0) + b))
end
