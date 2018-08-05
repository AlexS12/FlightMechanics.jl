# AIR CONSTANTS
module AirConstants

const GAMMA_AIR = 1.4  # Adiabatic index or ratio of specific heats (dry air at 20º C)
const R_AIR = 287.05287  # Specific gas constant for dry air (J/(Kg·K))

# Air at sea level conditions h=0 (m)
const RHO0 = 1.225  # Density at sea level (kg/m3)
const P0 = 101325.0  # Pressure at sea level (Pa)
const T0 = 288.15  # Temperature at sea level (K)
const A0 = 340.293990543  # Sound speed at sea level (m/s)
end


# EARTH CONSTANTS
module EarthConstants

const GRAVITY_ACCEL = 9.80665  # Gravity of Ethe Earth (m/s^2)
# Standard Gravitational Parameter
# product of the gravitational constant G and the mass M of the body
const STD_GRAV_PARAMETER = 3.986004418e14  # (m³/s²)
const EARTH_MASS = 5.9722e24  # Mass of the Earth (kg)
const GRAV_CONSTANT = 6.67384e11  # Gravitational constant (N·m²/kg²)
const EARTH_MEAN_RADIUS = 6371000  # Mean radius of the Earth (m)
end 


# CONVERSIONS
module ConversionConstants

const LB2KG = 0.453592  # Pounds (lb) to kilograms (kg)
const FT2M = 0.3048  # Feet (ft) to meters (m)
const SLUG2KG = 14.5939  # Slug to kilograms (kg)
const SLUGFT2_2_KGM2 = 1.35581795  # Slug*feet^2 to kilograms*meters^2 (kg*m^2)
const KT2MS = 0.514444  # knots (kt) to meters/second (m/s)
const MS2KT = 1.94384  # meters/second (m/s) to knots (kt)
end
