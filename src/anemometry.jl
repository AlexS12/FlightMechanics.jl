import .AirConstants: GAMMA_AIR, P0, RHO0


γ = GAMMA_AIR
p0 = P0
ρ0 = RHO0


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
    cas = sqrt((2*γ*p0) / ((γ-1) * ρ0) * ((qc/p0 + 1) ^(γ / (γ-1)) - 1))
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
function qc2tas(qc::Real, ρ::Real, p::Real)
    tas2 = (2*γ*p) / ((γ-1) * ρ0) * ((qc/p + 1) ^(γ / (γ-1)) - 1) * ρ0/ρ
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
    eas = sqrt((2*γ*p) / ((γ-1) * ρ0) * ((qc/p + 1) ^(γ / (γ-1)) - 1))
    return eas
end


"""
    tas2eas(qc::Real, ρ::Real)

Calculate true airspeed from equivalent airspeed

Implementation from:

.. [1] Ward, D. T. (1993). Introduction to flight test engineering. Elsevier
    Science Ltd. (page 13, formula 2.15)
"""
function tas2eas(tas, ρ)
    eas = tas * sqrt(ρ / ρ0)
    return eas
end


"""
    eas2tas(qc::Real, ρ::Real)

Calculate equivalent airspeed from true airspeed

Implementation from:

.. [1] Ward, D. T. (1993). Introduction to flight test engineering. Elsevier
    Science Ltd. (page 13, formula 2.15)
"""
function eas2tas(eas, ρ)
    tas = eas / sqrt(ρ / ρ0)
    return tas
end


function cas2eas(cas, ρ, p)
    
    return eas
end


function eas2cas(eas, ρ, p)

    return cas
end


function cas2tas(cas, ρ, p)

    return tas
end


function tas2cas(tas, ρ, p)

    return cas
end


function tas_alpha_beta_from_uvw(u, v, w)
    
    tas = sqrt(u*u + v*v + w*w)
    alpha = atan(w / u)
    beta = asin(v / TAS)
    return tas, alpha, beta
end


function incompressible_qinf(tas, ρ)
    return 0.5 * ρ * tas*tas
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
