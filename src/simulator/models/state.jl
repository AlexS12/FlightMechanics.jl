using FlightMechanics
using FlightMechanics.Simulator.Models

export State,
       Position, PositionLLH, PositionEarth, PositionECEF,
       get_position, get_llh, get_xyz_earth, get_xyz_ecef, get_height,
       get_euler_angles, get_quaternion,
       get_body_velocity, get_horizon_velocity,
       get_flight_path_angle,
       get_body_ang_velocity,
       get_turn_rate,
       get_body_accel,
       get_horizon_accel,
       get_body_ang_accel,
       get_sixdof_euler_fixed_mass_state


struct Position
    # latitude [rad], longitude [rad], height [m]
    llh::Array{T, 1} where T<:Number
    # x, y, z [m] Earth
    xyz_earth::Array{T, 1} where T<:Number
    # x, y, z [m] ECEF
    xyz_ecef::Array{T, 1} where T<:Number
end

# Constructors
PositionLLH(lat, lon, h, xe=0., ye=0., ze=0.) = Position([lat, lon, h], [xe, ye, ze], llh2ecef(lat, lon, h))
PositionECEF(x, y, z, xe=0., ye=0., ze=0.) = Position(ecef2llh(lat, lon, h), [xe, ye, ze], [x, y, z])
PositionEarth(x, y, z, lat=0., lon=0., h=0.) = Position([lat, lon, h], [x, y, z], llh2ecef(lat, lon, h))

get_llh(pos::Position) = pos.llh
get_xyz_earth(pos::Position) = pos.xyz_earth
get_xyz_ecef(pos::Position) = pos.xyz_ecef
get_height(pos::Position) = pos.llh[3]


struct State
    position::Position
    attitude::Attitude
    velocity::Array{T, 1} where T<:Number
    angular_velocity::Array{T, 1} where T<:Number
    acceleration::Array{T, 1} where T<:Number
    angular_acceleration::Array{T, 1} where T<:Number
end

# Position getters
get_position(state::State) = state.position
get_llh(state::State) = get_llh(state.position)
get_xyz_earth(state::State) = get_xyz_earth(state.position)
get_xyz_ecef(state::State) = get_xyz_ecef(state.position)
get_height(state::State) = get_height(state.position)
# Attitude getters
get_euler_angles(state::State) = get_euler_angles(state.attitude)
get_quaternion(state::State) = get_quaternion(state.attitude)
# Velocity getters
get_body_velocity(state::State) = state.velocity
get_horizon_velocity(state::State) = body2hor(get_body_velocity(state)...,
                                              get_quaternion(state)...)
function get_flight_paht_angle(state::State)
    vn, ve, vz = get_horizon_velocity(state)
    atan(-vz, norm([vn, ve]))
end

# Angular velocity getters
get_body_ang_velocity(state::State) = state.angular_velocity
get_turn_rate(state::State) = body2hor(get_body_ang_velocity(state)...,
                                       get_quaternion(state)...)[3]
# Acceleration getters
get_body_accel(state::State) = state.acceleration
get_horizon_accel(state::State) = body2hor(get_body_accel(state)...,
                                           get_quaternion(state)...)
# Angular acceleration getters
get_body_ang_accel(state::State) = state.angular_acceleration


function get_sixdof_euler_fixed_mass_state(state::State)
    [get_body_velocity(state)...,
     get_body_ang_velocity(state)...,
     get_euler_angles(state)...,
     get_xyz_earth(state)...]
end
