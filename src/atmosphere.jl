using FlightMechanics

export atmosphere_isa


const G0 = GRAVITY_ACCEL

"""
    atmosphere_isa(height)

Calculate temperature, pressure, density and sound velocity for the
given geopotential height according to International Standard Atmosphere 1976.

# References

- [1] U.S. Standard Atmosphere, 1976, U.S. Government Printing Office,
        Washington, D.C., 1976

From: https://en.wikipedia.org/wiki/U.S._Standard_Atmosphere

| Layer | h (m) | p (Pa)  | T (K)  | ``α`` (K/m) |
|-------|-------|---------|--------|-------------|
| 0     | 0     | 101325  | 288.15 | -0.0065     |
| 1     | 11000 | 22632.1 | 216.65 | 0           |
| 2     | 20000 | 5474.89 | 216.65 | 0.001       |
| 3     | 32000 | 868.019 | 228.65 | 0.0028      |
| 4     | 47000 | 110.906 | 270.65 | 0           |
| 5     | 51000 | 66.9389 | 270.65 | -0.0028     |
| 6     | 71000 | 3.95642 | 214.65 | -0.002      |
"""
function atmosphere_isa(height)

    if 0 <= height < 11000  # Troposphere
        alpha = -0.0065  # K/m
        T0 = 288.15      # K
        p0 = 101325.0    # Pa

        T = T0 + alpha * height
        p = p0 * (T0 / T) ^ (G0 / (R_AIR * alpha))

    elseif 11000 <= height < 20000  # Tropopause
        T = 216.65    # K
        p0 = 22632.1  # Pa
        h0 = 11000    # m

        p = p0 * exp(-G0 * (height - h0) / (R_AIR * T))

    elseif 20000 <= height < 32000  # Stratosphere 1
        alpha = 0.001    # K/m
        T0 = 216.65      # K
        p0 = 5474.89     # Pa
        h0 = 20000       # m

        T = T0 + alpha * (height - h0)
        p = p0 * (T0 / T) ^ (G0 / (R_AIR * alpha))

    elseif 32000 <= height < 47000  # Stratosphere 2
        alpha = 0.0028   # K/m
        T0 = 228.65      # K
        p0 = 868.019     # Pa
        h0 = 32000       # m

        T = T0 + alpha * (height - h0)
        p = p0 * (T0 / T) ^ (G0 / (R_AIR * alpha))

    elseif 47000 <= height < 51000  # Stratopause
        T = 270.65    # K
        p0 = 110.906  # Pa
        h0 = 47000    # m
        h0 = 47000    # m

        p = p0 * exp(-G0 * (height - h0) / (R_AIR * T))

    elseif 51000 <= height < 71000  # Mesosphere 1
        alpha = -0.0028  # K/m
        T0 = 270.65      # K
        p0 = 66.9389     # Pa
        h0 = 51000       # m

        T = T0 + alpha * (height - h0)
        p = p0 * (T0 / T) ^ (G0 / (R_AIR * alpha))

    elseif 71000 <= height <= 84500  # Mesosphere 2
        alpha = -0.002   # K/m
        T0 = 214.65      # K
        p0 = 3.95642     # Pa
        h0 = 71000       # m

        T = T0 + alpha * (height - h0)
        p = p0 * (T0 / T) ^ (G0 / (R_AIR * alpha))

    else
        throw(DomainError(height, "height must be between 0 m and 84500 m"))
    end

    rho = p / (R_AIR * T)
    a   = sqrt(GAMMA_AIR * R_AIR * T)

    return [T, p, rho, a]
end


# TODO: document difference in results
"""
    atmosphere_f16(height)

Standard Atmosphere implemented in Stevens F16 model.

# Arguments
- alt: altitude [m].

# Returns
- T: temperature [K].
- P: pressure [Pa].
- RHO: density [kg/m³].
- a: Speed of sound [m/s].

# Notes
- This model is implemented just for F16 complete model validation purposes.
- For ISA 1978 atmospheric model, use atmosphere_isa.
- Note limited precision in tests against `atmosphere_isa`.

# References
- [1] Stevens, B. L., Lewis, F. L., & Johnson, E. N. (2015). Aircraft control
 and simulation: dynamics, controls design, and autonomous systems. John Wiley
 & Sons. (page 715)
"""
function atmosphere_f16(alt)

    # The caluclations are prepared for input in ft
    ALT = alt * M2FT

    RO = 0.002377  # sluf/ft³
    TFAC = 1.0 - ALT*0.703E-05
    T = 519.0*TFAC  # ºR

    if ALT >= 35000.0 T=390.0 end
    RHO = RO*(TFAC^4.14)
    # will be calculated in aerostate
    # RMACH = VT/sqrt(1.4*1716.3*T)
    # QBAR = 0.5*RHO*VT*VT
    PS = 1715.0*RHO*T  # psf
    a = sqrt(1.4*1716.3*T)
    return [T*RANK2KEL, PS*PSF2PA, RHO*SLUGFT3_2_KGM3, a*FT2M]
end
