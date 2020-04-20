import Base.convert

abstract type DynamicSystem end


get_x(ds::DynamicSystem) = ds.x
get_x(ds::DynamicSystem, state::State) = get_x(convert(typeof(ds), state))
get_x_dot(ds::DynamicSystem) = ds.x_dot
get_x_dot(ds::DynamicSystem, state::State) = get_x(convert(typeof(ds), state))
get_state_equation(ds::DynamicSystem) = ds.state_equation
get_state_equation_ode_wrapper(ds::DynamicSystem) = ds.state_equation_ode_wrapper!

get_x_count(ds::DynamicSystem) = length(get_x(ds))
get_state(ds::DynamicSystem) = convert(State, ds)

set_x!(ds::DynamicSystem, x) = ds.x[:] = x
set_x_dot!(ds::DynamicSystem, x_dot) = ds.x_dot[:] = x_dot


# ---------------------- SixDOFEulerFixedMass ----------------------
struct SixDOFEulerFixedMass <: DynamicSystem
    x :: Array{T, 1} where T<:Number
    x_dot :: Array{T, 1} where T<:Number
    state_equation :: Function
    state_equation_ode_wrapper! :: Function
end

function six_dof_euler_fixed_mass_ode_wrapper!(dx, x, p, t)
    mass = p[1]
    inertia = p[2]
    forces = p[3]
    moments = p[4]
    h = p[5]
    dx[:] = six_dof_euler_fixed_mass(x, mass, inertia, forces, moments, h)
end

SixDOFEulerFixedMass(x, x_dot) = SixDOFEulerFixedMass(
    x,
    x_dot,
    six_dof_euler_fixed_mass,
    six_dof_euler_fixed_mass_ode_wrapper!
    )
SixDOFEulerFixedMass(x) = SixDOFEulerFixedMass(x, fill(NaN, 12))
SixDOFEulerFixedMass() = SixDOFEulerFixedMass(fill(NaN, 12))


function convert(::Type{SixDOFEulerFixedMass}, state::State)
    x = [
        get_body_velocity(state)...,
        get_body_ang_velocity(state)...,
        get_euler_angles(state)...,
        get_xyz_earth(state)...,
    ]

    x_dot = [
        get_body_accel(state)...,
        get_body_ang_accel(state)...,
        get_euler_angles_rates(state)...,
        get_horizon_velocity(state)...,
    ]

    SixDOFEulerFixedMass(x, x_dot)
end

function convert(state::State, ds::SixDOFEulerFixedMass)
    x = get_x(ds)
    x_dot = get_x_dot(ds)
    State(
        EarthPosition(x[10:12]..., get_position(state).ref_point),
        Attitude(x[7:9]...),
        x[1:3],
        x[4:6],
        x_dot[1:3],
        x_dot[4:6]
        )
end

# ---------------------- SixDOFQuaternionFixedMass ----------------------
struct SixDOFQuaternionFixedMass <: DynamicSystem
    x :: Array{T, 1} where T<:Number
    x_dot::Array{T, 1} where T<:Number
    state_equation :: Function
    state_equation_ode_wrapper! :: Function
    # TODO: must include k
    # k :: Number  # orthonormality error factor (see six_dof_quaternion_fixed_mass)
end

function six_dof_quaternion_fixed_mass_ode_wrapper!(dx, x, p, t, k=0.0)
    mass = p[1]
    inertia = p[2]
    forces = p[3]
    moments = p[4]
    h = p[5]
    dx[:] = six_dof_quaternion_fixed_mass(x, mass, inertia, forces, moments, h, k)
end

SixDOFQuaternionFixedMass(x, x_dot) = SixDOFQuaternionFixedMass(
    x,
    x_dot,
    six_dof_quaternion_fixed_mass,
    six_dof_quaternion_fixed_mass_ode_wrapper!
    )
SixDOFQuaternionFixedMass(x) = SixDOFQuaternionFixedMass(x, fill(NaN, 13))
SixDOFQuaternionFixedMass() = SixDOFQuaternionFixedMass(fill(NaN, 13))

function convert(state::State, ds::SixDOFQuaternionFixedMass)
    x = get_x(ds)
    x_dot = get_x_dot(ds)
    State(
        EarthPosition(x[11:13]..., get_position(state).ref_point), 
        Attitude(x[7:10]...), 
        x[1:3], 
        x[4:6], 
        x_dot[1:3], 
        x_dot[4:6]
        )
end

function convert(::Type{SixDOFQuaternionFixedMass}, state::State)
    x = [
        get_body_velocity(state)...,
        get_body_ang_velocity(state)...,
        get_quaternions(state)...,
        get_xyz_earth(state)...,
    ]
    
    x_dot = [
        get_body_accel(state)...,
        get_body_ang_accel(state)...,
        get_quaternions_rate(state)...,
        get_horizon_velocity(state)...,
    ]

    SixDOFQuaternionFixedMass(x, x_dot)
end
# ---------------------- SixDOFECEFQuaternionFixedMass ----------------------
struct SixDOFECEFQuaternionFixedMass <: DynamicSystem
    x :: Array{T, 1} where T<:Number
    x_dot::Array{T, 1} where T<:Number
    state_equation :: Function
    state_equation_ode_wrapper! :: Function
    # TODO: must include k
    # k :: Number  # orthonormality error factor (see six_dof_ecef_quaternion_fixed_mass)
end

function six_dof_ecef_quaternion_fixed_mass_ode_wrapper!(dx, x, p, t, k=0.0)
    mass = p[1]
    inertia = p[2]
    forces = p[3]
    moments = p[4]
    h = p[5]
    dx[:] = six_dof_ecef_quaternion_fixed_mass(x, mass, inertia, forces, moments, h, k)
end

SixDOFECEFQuaternionFixedMass(x, x_dot) = SixDOFECEFQuaternionFixedMass(
    x,
    x_dot,
    six_dof_ecef_quaternion_fixed_mass,
    six_dof_ecef_quaternion_fixed_mass_ode_wrapper!
    )
SixDOFECEFQuaternionFixedMass(x) = SixDOFECEFQuaternionFixedMass(x, fill(NaN, 13))
SixDOFECEFQuaternionFixedMass() = SixDOFECEFQuaternionFixedMass(fill(NaN, 13))

function convert(state::State, ds::SixDOFECEFQuaternionFixedMass)
    x = get_x(ds)
    x_dot = get_x_dot(ds)
    State(
        ECEFPosition(x[11:13]...),
        Attitude(x[7:10]...),
        x[1:3],
        x[4:6],
        x_dot[1:3],
        x_dot[4:6]
        )
end

function convert(::Type{SixDOFECEFQuaternionFixedMass}, state::State)
    x = [
        get_body_velocity(state)...,
        get_body_ang_velocity(state)...,
        get_quaternions(state)...,
        get_xyz_ecef(state)...,
    ]

    x_dot = [
        get_body_accel(state)...,
        get_body_ang_accel(state)...,
        get_quaternions_rate(state)...,
        get_ecef_velocity(state)...,
    ]

    SixDOFECEFQuaternionFixedMass(x, x_dot)
end


# ---------------------- SixDOFAeroEulerFixedMass ----------------------
