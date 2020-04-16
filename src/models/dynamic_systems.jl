import Base.convert

abstract type DynamicSystem end


get_state_equation(ds::DynamicSystem) = ds.state_equation
get_state_equation_ode_wrapper(ds::DynamicSystem) = ds.state_equation_ode_wrapper!
get_x(ds::DynamicSystem) = ds.x
get_x(ds::DynamicSystem, state::State) = get_x(convert(typeof(ds), state))
get_x_count(ds::DynamicSystem) = length(get_x(ds))
get_state(ds::DynamicSystem) = convert(State, ds)


# ---------------------- SixDOFEulerFixedMass ----------------------
struct SixDOFEulerFixedMass <: DynamicSystem
    x :: Array{T, 1} where T<:Number
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

SixDOFEulerFixedMass() = SixDOFEulerFixedMass(fill(NaN, 12), six_dof_euler_fixed_mass, six_dof_euler_fixed_mass_ode_wrapper!)
SixDOFEulerFixedMass(x) = SixDOFEulerFixedMass(x, six_dof_euler_fixed_mass, six_dof_euler_fixed_mass_ode_wrapper!)

function convert(::Type{SixDOFEulerFixedMass}, state::State)
    x = [
            get_body_velocity(state)...,
            get_body_ang_velocity(state)...,
            get_euler_angles(state)...,
            get_xyz_earth(state)...,
        ]

    SixDOFEulerFixedMass(x)
end

function convert(::Type{State}, ds::SixDOFEulerFixedMass)
    # TODO: add x_dot to Dynamic system to fill accel and ang_accel
    x = get_x(ds)
    # TODO: only PositionEarth is being updated. What about LLH or ECEF...
    State(PositionEarth(x[10:12]...), Attitude(x[7:9]...), x[1:3], x[4:6], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
end

# TODO: implementations for the rest of the systems
# ---------------------- SixDOFQuaternionFixedMass ----------------------
struct SixDOFQuaternionFixedMass <: DynamicSystem
    x :: Array{T, 1} where T<: Number
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

SixDOFQuaternionFixedMass() = SixDOFQuaternionFixedMass(fill(NaN, 13), six_dof_quaternion_fixed_mass, six_dof_quaternion_fixed_mass_ode_wrapper!)
SixDOFQuaternionFixedMass(x) = SixDOFQuaternionFixedMass(x, six_dof_quaternion_fixed_mass, six_dof_quaternion_fixed_mass_ode_wrapper!)

function convert(::Type{State}, ds::SixDOFQuaternionFixedMass)
    # TODO: add x_dot to Dynamic system to fill accel and ang_accel
    x = get_x(ds)
    # TODO: only PositionEarth is being updated. What about LLH or ECEF...
    State(PositionEarth(x[11:13]...), Attitude(x[7:10]...), x[1:3], x[4:6], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
end

function convert(::Type{SixDOFQuaternionFixedMass}, state::State)
    x = [
            get_body_velocity(state)...,
            get_body_ang_velocity(state)...,
            get_quaternions(state)...,
            get_xyz_earth(state)...,
        ]

    SixDOFQuaternionFixedMass(x)
end
# ---------------------- SixDOFECEFQuaternionFixedMass ----------------------

# ---------------------- SixDOFAeroEulerFixedMass ----------------------
