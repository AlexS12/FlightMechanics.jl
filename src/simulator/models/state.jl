using FlightMechanics.Simulator

export State,
       EarthBodyState,
       get_euler_angles,
       get_quaternion,
       get_earth_position,
       get_body_velocity,
       get_horizon_velocity,
       get_body_ang_velocity,
       get_body_accel,
       get_horizon_accel,
       get_body_ang_accel,
       get_sixdof_euler_fixed_mass_state,


abstract type State end


struct EarthBodyState<:State
    # TODO: A position struct here is necessary to genrealize different
    # representations: Earth, LLA
    # In that case maybe this specific State is not necessary.
    xe::Number
    ye::Number
    ze::Number
    u::Number
    v::Number
    w::Number
    p::Number
    q::Number
    r::Number
    att::Attitude
    u_dot::Number
    v_dot::Number
    w_dot::Number
    p_dot::Number
    q_dot::Number
    r_dot::Number
end


get_euler_angles(state::State) = get_euler_angles(state.att)
get_quaternion(state::State) = get_quaternion(state.att)

get_earth_position(state::EarthBodyState) = [state.xe, state.ye, state.ze]
get_height(state::EarthBodyState) = -state.ze

get_body_velocity(state::EarthBodyState) = [state.u, state.v, state.w]
get_horizon_velocity(state::EarthBodyState) = body2hor(state.u, state.v, state.w,
                                                       get_quaternion(state.att)...)

get_body_ang_velocity(state::EarthBodyState) = [state.p, state.q, state.r]

get_body_accel(state::EarthBodyState) = [state.u_dot, state.v_dot, state.w_dot]
get_horizon_accel(state::EarthBodyState) = body2hor(state.u_dot, state.v_dot, state.w_dot...,
                                                    get_quaternion(state.att)...)

get_body_ang_accel(state::EarthBodyState) = [state.p_dot, state.q_dot, state.r_dot]


function get_sixdof_euler_fixed_mass_state(state::EarthBodyState)
    [state.u, state.v, state.w,
     state.p, state.q, state.r,
     get_euler_angles(state)...,
     state.xe, state.ye, state.ze]
end
