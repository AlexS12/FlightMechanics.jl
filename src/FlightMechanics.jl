module FlightMechanics

export
# Air Constants
GAMMA_AIR, R_AIR, RHO0, P0, T0, A0,
# Earth constants
GRAVITY_ACCEL, STD_GRAV_PARAMETER, GRAV_CONSTANT, ROT_VELOCITY, EARTH_MASS,
EARTH_MEAN_RADIUS,
# Ellipsoids
Ellipsoid,
Clarke1866, Clarke1880,
International, Bessel, Everest, ModifiedEverest,
AustralianNational, SouthAmerican1969, Airy, ModifiedAiry, Hough,
Fischer1960SouthAsia, Fischer1960Mercury, Fischer1968,
WGS60, WGS66, WGS72, WGS84,
# Conversion constants
LB2KG, KG2LB, FT2M, M2FT, IN2M, M2IN, SLUG2KG, SLUGFT2_2_KGM2, SLUGFT3_2_KGM3,
KT2MS, MS2KT, DEG2RAD, RAD2DEG, HP2WAT, WAT2HP, LBF2N, PA2PSF, PSF2PA,
RANK2KEL, KEL2RANK,
# Coordinates
body2hor, rot_matrix_body2hor, hor2body, rot_matrix_hor2body,
wind2hor, hor2wind, body2wind, wind2body,
ecef2hor, rot_matrix_ecef2hor, hor2ecef, rot_matrix_hor2ecef,
body2ecef, rot_matrix_body2ecef, ecef2body, rot_matrix_ecef2body,
quaternion2euler, euler2quaternion,
llh2ecef, ecef2llh,
# atmosphere
atmosphere_isa, atmosphere_f16

include("constants.jl")
include("atmosphere.jl")
include("coordinates.jl")
include("anemometry.jl")
include("mechanics.jl")
include("flight_mechanics.jl")

# 6 DOF dynamic models
include("dynamics/sixdof_euler_fixed_mass.jl")
include("dynamics/sixdof_quaternion_fixed_mass.jl")
include("dynamics/sixdof_ecef_quaternion_fixed_mass.jl")

# Models
include("models/Models.jl")
# Aircrafts
include("aircrafts/Aircrafts.jl")

end # module
