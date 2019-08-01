module Models

import Base: +, -, *, isapprox
using LinearAlgebra
using FlightMechanics

export
# Attitude
Attitude, get_euler_angles, get_quaternions,
# Position
Position, PositionLLH, PositionEarth, PositionECEF,
get_position, get_llh, get_xyz_earth, get_xyz_ecef, get_height,
# State
State,
get_attitude,
get_body_velocity, get_horizon_velocity, get_flight_path_angle,
get_body_ang_velocity, get_turn_rate,
get_body_accel, get_horizon_accel,
get_body_ang_accel,
get_sixdof_euler_fixed_mass_state,
# Mass
RigidSolid, PointMass, get_mass, get_cg, get_inertia,
# Pont Forces and moments
PointForcesMoments, translate_forces_moments, rotate,
# Atmosphere
AtmosphereISA,AtmosphereF16,
get_temperature, get_pressure, get_density, get_sound_velocity,
calculate_atmosphere,
# Wind
Wind, ConstantWind,
get_wind_NED, get_wind_body, get_wind_direction, get_wind_intensity,
get_vertical, calculate_wind,
# Gravity
EarthConstantGravity, get_gravity_horizon, get_gravity_accel, get_gravity_body,
calculate_gravity,
# Environment
Environment, calculate_environment, get_atmos, get_wind, get_gravity,
# Aerostate
AeroState, get_alpha, get_beta, get_aero_angles, get_alpha_dot, get_tas,
get_eas, get_cas, get_ias, get_aero_speeds, get_qinf, get_mach,
generate_state_aerostate, check_state_aerostate_env_coherence


include("attitude.jl")
include("position.jl")
include("state.jl")
include("mass.jl")
include("point_forces_moments.jl")
include("atmosphere.jl")
include("wind.jl")
include("gravity.jl")
include("environment.jl")
include("aero_state.jl")
    include("controls.jl")
    include("fcs.jl")
    include("propulsion.jl")
    include("aerodynamics.jl")
    include("aircraft.jl")
end
