# AIR CONSTANTS
export GAMMA_AIR, R_AIR, RHO0, P0, T0, A0

export GRAVITY_ACCEL, STD_GRAV_PARAMETER, GRAV_CONSTANT, ROT_VELOCITY,
      EARTH_MASS, EARTH_MEAN_RADIUS,
      Clarke1866, Clarke1880,
      International,
      Bessel,
      Everest,
      ModifiedEverest,
      AustralianNational,
      SouthAmerican1969,
      Airy,
      ModifiedAiry,
      Hough,
      Fischer1960SouthAsia, Fischer1960Mercury, Fischer1968,
      WGS60, WGS66, WGS72, WGS84

export LB2KG,
      FT2M,
      IN2M, M2IN,
      SLUG2KG,
      SLUGFT2_2_KGM2,
      KT2MS, MS2KT,
      DEG2RAD, RAD2DEG,
      HP2WAT, WAT2HP

const GAMMA_AIR = 1.4  # Adiabatic index or ratio of specific heats (dry air at 20º C)
const R_AIR = 287.05287  # Specific gas constant for dry air (J/(Kg·K))

# Air at sea level conditions h=0 (m)
const RHO0 = 1.225  # Density at sea level (kg/m3)
const P0 = 101325.0  # Pressure at sea level (Pa)
const T0 = 288.15  # Temperature at sea level (K)
const A0 = 340.293990543  # Sound speed at sea level (m/s)


# EARTH CONSTANTS

const GRAVITY_ACCEL = 9.80665  # Gravity of Ethe Earth (m/s^2)
# Standard Gravitational Parameter
# product of the gravitational constant G and the mass M of the body
# TODO: review values with
# Rogers, R. M. (2007). Applied mathematics in integrated navigation systems.
# American Institute of Aeronautics and Astronautics. (Page 76, table 4.1)
const ROT_VELOCITY = 7.292115e-5  # Angular velocity ω_i/e (rad/s)
const STD_GRAV_PARAMETER = 3.986004418e14  # (m³/s²)
const EARTH_MASS = 5.9722e24  # Mass of the Earth (kg)
const GRAV_CONSTANT = 6.67384e11  # Gravitational constant (N·m²/kg²)
const EARTH_MEAN_RADIUS = 6371000  # Mean radius of the Earth (m)

# Ellipsoids
struct Ellipsoid
    a :: Real  # semi-major axis
    finv :: Real  # inverse of flattening
    f :: Real  # flattening
    b :: Real  # semi-minor axis
    e2 :: Real  # eccentricity squared
    ϵ2 :: Real  # second eccentricity squared
end


Ellipsoid(a, finv) = begin
    f = 1.0 / finv
    b = a * (1.0 - f)
    e2 = 1.0 - (1.0 - f) ^ 2.0
    ϵ2 = a*a / (b*b) - 1.0
    Ellipsoid(a, finv, f, b, e2, ϵ2)
end

# Rogers, R. M. (2007). Applied mathematics in integrated navigation systems.
# American Institute of Aeronautics and Astronautics. (Page 76, table 4.1)
Clarke1866           = Ellipsoid(6378206.4  , 294.9786982)
Clarke1880           = Ellipsoid(6378249.145, 294.465)
International        = Ellipsoid(6378388.0  , 297.0)
Bessel               = Ellipsoid(6377397.155, 299.1528128)
Everest              = Ellipsoid(6377276.345, 300.8017)
ModifiedEverest      = Ellipsoid(6377304.063, 300.8017)
AustralianNational   = Ellipsoid(6378160.0  , 298.25)
SouthAmerican1969    = Ellipsoid(6378160.0  , 298.25)
Airy                 = Ellipsoid(6377564.396, 299.3249646)
ModifiedAiry         = Ellipsoid(6377340.189, 299.3249646)
Hough                = Ellipsoid(6378270.0  , 297.0)
Fischer1960SouthAsia = Ellipsoid(6378155.0  , 298.3)
Fischer1960Mercury   = Ellipsoid(6378166.0  , 298.3)
Fischer1968          = Ellipsoid(6378150.0  , 298.3)
WGS60                = Ellipsoid(6378165.0  , 298.3)
WGS66                = Ellipsoid(6378145.0  , 298.25)
WGS72                = Ellipsoid(6378135.0  , 298.26)
WGS84                = Ellipsoid(6378137.0  , 298.257223563)


# CONVERSIONS
const LB2KG = 0.453592  # Pounds (lb) to kilograms (kg)
const FT2M = 0.3048  # Feet (ft) to meters (m)
const IN2M = 0.0254  # Inches (in) to meters (m)
const M2IN = 39.37007874  # meters (m) to inches(in)
const SLUG2KG = 14.5939  # Slug to kilograms (kg)
const SLUGFT2_2_KGM2 = 1.35581795  # Slug*feet^2 to kilograms*meters^2 (kg*m^2)
const KT2MS = 0.514444  # knots (kt) to meters/second (m/s)
const MS2KT = 1.94384  # meters/second (m/s) to knots (kt)
const DEG2RAD = pi / 180  # degrees to radians
const RAD2DEG = 180 / pi  # radians to degrees
const HP2WAT = 745.7  # horse power to Watts
const WAT2HP = 0.00134102  # Wat to horse power
