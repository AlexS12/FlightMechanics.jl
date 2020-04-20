struct State
    position::Position
    attitude::Attitude
    velocity::Array{T, 1} where T<:Number  # body (u, v, w) in m/s
    angular_velocity::Array{T, 1} where T<:Number  # body (p, q, r) in rad/s
    acceleration::Array{T, 1} where T<:Number  # body (ax, ay, az) in m/s^2
    angular_acceleration::Array{T, 1} where T<:Number  # body (p_dot, q_dot, r_dot) in rad/s^2
end


# Position getters
get_position(state::State) = state.position
get_llh(state::State) = get_llh(state.position)
get_xyz_earth(state::State) = get_xyz_earth(state.position)
get_xyz_ecef(state::State) = get_xyz_ecef(state.position)
get_height(state::State) = get_height(state.position)

# Attitude getters
get_attitude(state::State) = state.attitude
get_euler_angles(state::State) = get_euler_angles(state.attitude)
get_quaternions(state::State) = get_quaternions(state.attitude)

# Velocity getters
get_body_velocity(state::State) = state.velocity
get_horizon_velocity(state::State) = body2hor(
    get_body_velocity(state)...,
    get_quaternions(state)...,
)
get_ecef_velocity(state::State) = body2ecef(
    get_body_velocity(state)...,
    get_llh(state)[1:2]...,
    get_euler_angles(state)...,
)

function get_flight_path_angle(state::State)
    vn, ve, vz = get_horizon_velocity(state)
    atan(-vz, norm([vn, ve]))
end

# Angular velocity getters
get_body_ang_velocity(state::State) = state.angular_velocity
get_turn_rate(state::State) = body2hor(
    get_body_ang_velocity(state)...,
    get_quaternions(state)...,
)[3]
get_euler_angles_rates(state::State) = body_angular_velocity_to_euler_angles_rates(
    get_body_ang_velocity(state)...,
    get_euler_angles(state)[2:3]...,
)
get_quaternions_rate(state::State) = body_angular_velocity_to_quaternion_rates(
    get_body_ang_velocity(state)...,
    get_quaternions(state)...
)

# Acceleration getters
get_body_accel(state::State) = state.acceleration
get_horizon_accel(state::State) = body2hor(
    get_body_accel(state)...,
    get_quaternions(state)...,
)

# Angular acceleration getters
get_body_ang_accel(state::State) = state.angular_acceleration


function isapprox(x::State, y::State; rtol=1e-8, atol=0.0, nans=false)

    result = all([
        isapprox(x.position, y.position; rtol=rtol, atol=atol, nans=nans),
        isapprox(x.attitude, y.attitude; rtol=rtol, atol=atol, nans=nans),
        isapprox(x.velocity, y.velocity; rtol=rtol, atol=atol, nans=nans),
        isapprox(x.angular_velocity, y.angular_velocity, rtol=rtol, atol=atol, nans=nans),
        isapprox(x.acceleration, y.acceleration, rtol=rtol, atol=atol, nans=nans),
        isapprox(x.angular_acceleration, y.angular_acceleration, rtol=rtol, atol=atol, nans=nans),
    ])

    return result
end
