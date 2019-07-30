module Models

using FlightMechanics
import Base: +

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
RigidSolid, PointMass, get_mass, get_cg, get_inertia


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
