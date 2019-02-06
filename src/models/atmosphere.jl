using FlightMechanics
using FlightMechanics.Models

export AtmosphereISA,
       get_temperature, get_pressure, get_density, get_sound_velocity,
       calculate_atmosphere


"""
    Atmosphere

Atmospheric information at a given altitude.
"""
abstract type Atmosphere end

get_temperature(atmos::Atmosphere) = atmos.temperature
get_pressure(atmos::Atmosphere) = atmos.pressure
get_density(atmos::Atmosphere) = atmos.density
get_sound_velocity(atmos::Atmosphere) = atmos.sound_velocity


"""
    AtmosphereISA(temperature, pressure, density, sound_velocity)

International Standard Atmosphere 1976.

# See also

    atmosphere_isa(height)

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
struct AtmosphereISA<:Atmosphere
    temperature::Number  # K
    pressure::Number  # Pa
    density::Number  # kg/m³
    sound_velocity::Number  # m/s
end

# Sea level
AtmosphereISA(pos::Position) = AtmosphereISA(atmosphere_isa(get_height(pos))...)

function calculate_atmosphere(atmos::AtmosphereISA, pos::Position)
    AtmosphereISA(atmosphere_isa(get_height(pos))...)
end

# Not exported because it is used only for validation
import FlightMechanics: atmosphere_f16

"""
Standard Atmosphere implemented in Stevens F16 model.

# Notes
- This model is implemented just for F16 complete model validation purposes.
- For ISA 1978 atmospheric model, use AtmosphereISA.
- Note limited precision in tests against `atmosphere_isa`.

# References
- [1] Stevens, B. L., Lewis, F. L., & Johnson, E. N. (2015). Aircraft control
 and simulation: dynamics, controls design, and autonomous systems. John Wiley
 & Sons. (page 715)
"""
struct AtmosphereF16<:Atmosphere
    temperature::Number  # K
    pressure::Number  # Pa
    density::Number  # kg/m³
    sound_velocity::Number  # m/s
end

AtmosphereF16(pos::Position) = AtmosphereF16(atmosphere_f16(get_height(pos))...)

function calculate_atmosphere(atmos::AtmosphereF16, pos::Position)
    AtmosphereF16(atmosphere_f16(get_height(pos))...)
end
