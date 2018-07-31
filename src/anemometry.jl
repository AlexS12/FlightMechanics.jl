import .AirConstants: GAMMA_AIR, P0, RHO0, T0, A0


γ = GAMMA_AIR
p0 = P0
ρ0 = RHO0
a0 = A0


"""
    qc2cas(qc)

Calculate calibrated airspeed from ASI (Air Speed indicator), differential
pressure between impact pressure and static pressure.
qc = p_t - p_s

Implementation from:

.. [1] Ward, D. T. (1993). Introduction to flight test engineering. Elsevier
    Science Ltd. (page 13, formula 2.13)
"""
function qc2cas(qc)
    cas = sqrt((2*γ*p0) / ((γ-1) * ρ0) * ((qc/p0 + 1) ^(γ / (γ-1)) - 1))
    return cas
end


"""
    qc2tas(qc)

Calculate true airspeed from ASI (Air Speed indicator), differential
pressure between impact pressure and static pressure (qc = p_t - p_s), rho
and p.

Implementation from:

.. [1] Ward, D. T. (1993). Introduction to flight test engineering. Elsevier
    Science Ltd. (page 12, based on formula 2.11)
"""
function qc2tas(qc, ρ, p)
    tas2 = (2*γ*p) / ((γ-1) * ρ0) * ((qc/p + 1) ^(γ / (γ-1)) - 1) * ρ0/ρ
    tas = sqrt(tas2)
    return tas
end

"""
    qc2eas(qc, p)

Calculate equivalent airspeed from ASI (Air Speed indicator), differential
pressure between impact pressure and static pressure (qc = p_t - p_s) and p.

Implementation from:

.. [1] Ward, D. T. (1993). Introduction to flight test engineering. Elsevier
    Science Ltd.
"""
function qc2eas(qc, p)
    eas = sqrt((2*γ*p) / ((γ-1) * ρ0) * ((qc/p + 1) ^(γ / (γ-1)) - 1))
    return eas
end


"""
    tas2eas(tas, ρ)

Calculate equivalent airspeed from true airspeed and density at current 
altitude (ρ).

Implementation from:

.. [1] Ward, D. T. (1993). Introduction to flight test engineering. Elsevier
    Science Ltd. (page 13, formula 2.15)
"""
function tas2eas(tas, ρ)
    eas = tas * sqrt(ρ / ρ0)
    return eas
end


"""
    eas2tas(qc, ρ)

Calculate true airspeed from equivalent airspeed and density at current 
altitude (ρ).

Implementation from:

.. [1] Ward, D. T. (1993). Introduction to flight test engineering. Elsevier
    Science Ltd. (page 13, formula 2.15)
"""
function eas2tas(eas, ρ)
    tas = eas / sqrt(ρ / ρ0)
    return tas
end


"""
    cas2eas(cas, ρ, p)

Calculate equivalent airspeed from calibrated airspeed, density (ρ) and
pressure (p) at the current altitude.
"""
function cas2eas(cas, ρ, p)
    tas = cas2tas(cas, ρ, p)
    eas = tas2eas(tas, ρ)
    return eas
end


"""
    eas2cas(eas, ρ, p)

Calculate calibrated airspeed from equivalent airspeed, density (ρ) and
pressure (p) at the current altitude.
"""
function eas2cas(eas, ρ, p)
    tas = eas2tas(eas, ρ)
    cas = tas2cas(tas, ρ, p)
    return cas
end


"""
    cas2tas(cas, ρ, p)

Calculate true airspeed from calibrated airspeed, density (ρ) and pressure (p)
at the current altitude.
"""
function cas2tas(cas, ρ, p)
    
    a = sqrt(γ * p / ρ)

    temp = (cas*cas * (γ - 1.0)/(2.0 * a0*a0) + 1.0) ^ (γ / (γ - 1.0))
    temp = (temp - 1.0) * (p0 / p)
    temp = (temp + 1.0) ^ ((γ - 1.0) / γ) - 1.0

    tas = sqrt(2.0 * a*a / (γ - 1.0) * temp)

    return tas
end


"""
    cas2tas(cas, ρ, p)

Calculate true airspeed from calibrated airspeed, density (ρ) and pressure (p)
at the current altitude.
"""
function tas2cas(tas, ρ, p)
    
    a = sqrt(γ * p / ρ)

    temp = (tas*tas * (γ - 1.0)/(2.0 * a*a) + 1.0) ^ (γ / (γ - 1.0))
    temp = (temp - 1.0) * (p / p0) 
    temp = (temp + 1.0) ^ ((γ - 1.0) / γ) - 1.0
    
    cas = sqrt(2.0 * a0*a0 / (γ - 1) * temp)

    return cas
end


"""
    tas_alpha_beta_from_uvw(u, v, w)

Calculate true air speed (TAS), angle of attack (α) and angle of side-slip (β)
from aerodynamic velocity expressed in body axis.

Implementation from:

.. [1] Etkin, B. (2005). Dynamics of atmospheric flight. Dover Publications 
    (page 114, formulas 4.3,2 and 4.3,3)
"""
function tas_alpha_beta_from_uvw(u, v, w)
    
    tas = sqrt(u*u + v*v + w*w)
    alpha = atan(w / u)
    beta = asin(v / TAS)
    return [tas, alpha, beta]
end


"""
    incompressible_qinf(tas, ρ)

Calculate incompressible dynamic pressure from true airspeed (tas) and density
(ρ) at current altitude.

.. [1] Ward, D. T. (1993). Introduction to flight test engineering. Elsevier
    Science Ltd. (page 13, formula 2.14)
"""
function incompressible_qinf(tas, ρ)
    return 0.5 * ρ * tas*tas
end


"""
    compressible_qinf(tas, p, a)

Calculate compressible dynamic pressure from true airspeed (tas), static 
pressure (p) and sound velocity at current altitude.

Two different models are used depending on the Mach number:

- Subsonic case: Bernouilli's equation compressible form.
- Supersonic case: to be implemented.

.. [1] Ward, D. T. (1993). Introduction to flight test engineering. Elsevier
    Science Ltd. (page 12)
"""
function compressible_qinf(tas, p, a)

    M = tas / a

    if M < 1
        pt = p * (1 + (γ - 1) / 2 * M*M) ^ (γ / (γ - 1))
    else
        # TODO: implementation for M > 1
        error("Not Implemented yet")
    end
    return pt
end
