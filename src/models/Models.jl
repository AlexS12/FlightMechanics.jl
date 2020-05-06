module Models

import Base: +, -, *, isapprox, convert, copy, show
using LinearAlgebra
using FlightMechanics

export
# Attitude
Attitude, get_euler_angles, get_quaternions,
# Position
Position, PositionLLH, PositionEarth, PositionECEF,
LLHPosition, ECEFPosition, EarthPosition,
get_position, get_llh, get_xyz_earth, get_xyz_ecef, get_height,
# State
State,
get_attitude,
get_body_velocity, get_horizon_velocity, get_flight_path_angle,
get_body_ang_velocity, get_turn_rate,
get_body_accel, get_horizon_accel,
get_body_ang_accel,
# Mass
RigidSolid, PointMass, get_mass, get_cg, get_inertia,
# Pont Forces and moments
PointForcesMoments, translate_forces_moments, rotate,
get_point, get_forces, get_moments,
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
generate_state_aerostate, check_state_aerostate_env_coherence,
# Controls
Control, RangeControl, DiscreteControl,
get_value, get_value_range, get_value_choices, set_value!,
# Inputs
Input, ConstantInput, StepInput, DoubletInput, InverseDoubletInput, RampInput,
SinusoidalInput,
get_value,
# FCS
FCS,
set_stick_lon!, set_stick_lat!, set_pedals!, set_thtl!,
get_allow_out_of_range_inputs, get_throw_error_on_out_of_range_inputs,
set_allow_out_of_range_inputs!, set_throw_error_on_out_of_range_inputs!,
get_controls_ranges_trimmer,
Controls, StickPedalsLeverControls,
set_controls!, 
get_controls_array, get_n_controls, get_stick_lon, get_stick_lat, get_pedals, get_thrust_lever,
ControlsStream, StickPedalsLeverStream,
get_controls,
# Engine
Engine, get_engine_position, get_engine_orientation, get_engine_gyro_effects,
# Propulsion
Propulsion, get_propulsion_position, get_fuel_mass_props, calculate_propulsion,
get_engines, get_gyro_effects,
# Aerodynamics
Aerodynamics, aerodynamics_from_wind_total, aerodynamics_from_wind_coeff,
aerodynamics_from_body_total, aerodynamics_from_body_coeff, get_pfm,
get_wind_pfm, get_wind_adim_pfm, get_body_pfm, get_body_adim_pfm,
calculate_aerodynamics,
# Aircraft
Aircraft, get_mass_props, get_pfm, get_aerodynamics, get_propulsion, get_fcs, get_name,
get_wing_area, get_wing_span, get_chord, get_arp, get_empty_mass_props,
get_payload_mass_props, calculate_aircraft,
# DynamicSystem
SixDOFEulerFixedMass, SixDOFQuaternionFixedMass, SixDOFECEFQuaternionFixedMass,
SixDOFAeroEulerFixedMass,
get_state_equation, get_state_equation_ode_wrapper, get_x, get_x_count, get_state

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
include("inputs.jl")
include("fcs.jl")
include("propulsion.jl")
include("aerodynamics.jl")
include("aircraft.jl")
include("dynamic_systems.jl")
end
