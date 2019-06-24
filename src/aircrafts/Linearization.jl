using FlightMechanics
using FlightMechanics.Models
using FlightMechanics.Aircrafts

export Linearize, get_A, get_B, get_named_A, get_named_B,
        get_lon_A, get_lon_B, get_lat_A, get_lat_B


struct Linearize
    A :: Array{T, 2} where T<:Number
    B :: Array{T, 2} where T<:Number
    # C :: Array{T, 2} where T<:Number
    # D :: 0  # Array{T, 2} where T<:Number   # no feed forward
end


function Linearize(trimmed_ac::Aircraft, trimmed_state::State, trimmed_aerostate::AeroState, 
					trimmed_fcs::FCS, env::Environment, dx=0.001::Float64, du=0.001::Float64, tol=1e-6::Float64)
    
    grav = get_gravity(env)

    ## Cacluating A_matrix
    A = A_matrix_calc(trimmed_ac, trimmed_state, trimmed_aerostate, trimmed_fcs, env, grav, dx, tol)

    ## Cacluating B_matrix
    B = B_matrix_calc(trimmed_ac, trimmed_state, trimmed_aerostate, trimmed_fcs, env, grav, du, tol)

    Linearize(A, B)
end


get_A(linear_sys::Linearize) = linear_sys.A
get_B(linear_sys::Linearize) = linear_sys.B

state_names = ["u", "v", "w", "p", "q", "r", "psi", "theta", "psi", "xe", "ye", "ze"]
control_names = ["delta_e", "delta_a", "delta_r", "delta_t"]
function get_named_A(linear_sys::Linearize)
    A = copy(linear_sys.A)
    convert(Any, A)
    A = hcat(["u", "v", "w", "p", "q", "r", "psi", "theta", "psi", "xe", "ye", "ze"], A)
    A = vcat(reshape([" ", "u", "v", "w", "p", "q", "r", "psi", "theta", "psi", "xe", "ye", "ze"], 1, 13), A)
    return A
end
function get_named_B(linear_sys::Linearize)
    B = copy(linear_sys.B)
    convert(Any, B)
    B = hcat(["u", "v", "w", "p", "q", "r", "psi", "theta", "psi", "xe", "ye", "ze"], B)
    B = vcat(reshape([" ", "delta_e", "delta_a", "delta_r", "delta_t"], 1, 5), B)
    return B
end

function get_lon_A(linear_sys::Linearize)
    A = copy(linear_sys.A)
    A = [
        A[1,1] A[3,1] A[5,1] A[8,1] A[10,1] A[12,1];
        A[1,3] A[3,3] A[5,3] A[8,3] A[10,3] A[12,3];
        A[1,5] A[3,5] A[5,5] A[8,5] A[10,5] A[12,5];
        A[1,8] A[3,8] A[5,8] A[8,8] A[10,8] A[12,8];
        A[1,10] A[3,10] A[5,10] A[8,10] A[10,10] A[12,10];
        A[1,12] A[3,12] A[5,12] A[8,12] A[10,12] A[12,12]
        ]

    return Array(transpose(A))
end

function get_lon_B(linear_sys::Linearize)
    B = copy(linear_sys.B)
    B = [
        B[1,1] B[1,4];
        B[3,1] B[3,4];
        B[5,1] B[5,4];
        B[8,1] B[8,4];
        B[10,1] B[10,4];
        B[12,1] B[12,4];
        ]

return B
end

function get_lat_A(linear_sys::Linearize)
    A = copy(linear_sys.A)
    A = [
        A[2,2] A[4,2] A[6,2] A[7,2] A[9,2] A[11,2];
        A[2,4] A[4,4] A[6,4] A[7,4] A[9,4] A[11,4];
        A[2,6] A[4,6] A[6,6] A[7,6] A[9,6] A[11,6];
        A[2,7] A[4,7] A[6,7] A[7,7] A[9,7] A[11,7];
        A[2,9] A[4,9] A[6,9] A[7,9] A[9,9] A[11,9];
        A[2,11] A[4,11] A[6,11] A[7,11] A[9,11] A[11,11];
        ]

    return Array(transpose(A))
end

function get_lat_B(linear_sys::Linearize)
    B = copy(linear_sys.B)
    B = [
        B[2,2] B[2,3];
        B[4,2] B[4,3];
        B[6,2] B[6,3];
        B[7,2] B[7,3];
        B[9,2] B[9,3];
        B[11,2] B[11,3]
        ]

return B
end


function A_matrix_calc(trimmed_ac, trimmed_state, trimmed_aerostate, trimmed_fcs, env, grav, dx_, tol)

    x = get_sixdof_euler_fixed_mass_state(trimmed_state)
    n = length(x)

    dx = dx_.*x      # purturbation in all state variables
    for i = 1:n   # set perturbations
        if dx[i] == 0.0
            dx[i] = dx_
            if i == n    # 12th state => ze should be negative
                dx[i] = -dx_
            end
        end
    end

    last = zeros(Float64, n,1)  # single column from A_matrix
    A = zeros(Float64, n,n)

    for j = 1:n
        xt = x
        for i = 1:10
            xt[j] = x[j] + dx[j]
            mass, inertia, forces, moments = recalculate_aircraft(xt, trimmed_ac, trimmed_fcs, grav)
            xd1 = six_dof_euler_fixed_mass(xt, mass, inertia, forces, moments)

            xt[j] = x[j] - dx[j]
            mass, inertia, forces, moments = recalculate_aircraft(xt, trimmed_ac, trimmed_fcs, grav)
            xd2 = six_dof_euler_fixed_mass(xt, mass, inertia, forces, moments)

            A[:, j] = (transpose(xd1 - xd2) ./ (2*dx[j]))
            if false in (max.(abs.(A[:,j] - last) ./ abs.(A[:,j] .+ 1e-12)) .< tol)
                break
            end
            dx[j] = 0.5*dx[j]
            last = A[:,j]
            iteration = i
            if iteration == 10
                println("not converged on A, column $j")
            end
        end
    end

    A_ = A.*2
    return A_
end


function B_matrix_calc(trimmed_ac, trimmed_state, trimmed_aerostate, trimmed_fcs, env, grav, du_, tol)

    x = get_sixdof_euler_fixed_mass_state(trimmed_state)
    n = length(x)
    u = get_controls(trimmed_fcs)
    m = length(u)

    du = du_.*u
    for i = 1:m
        if du[i] == 0.0
            du[i] = du_
        end
    end

    last = zeros(Float64, n,1)  # last column of matrix
    B = zeros(Float64, n,m)

    for j = 1:m
        ut = u
        for i = 1:10
            ut[j] = u[j] + du[j]    # adding du to u
            fcs = fcs_from_u(trimmed_fcs, ut)
            mass, inertia, forces, moments = recalculate_aircraft(x, trimmed_ac, trimmed_fcs, grav)
            xd1 = six_dof_euler_fixed_mass(x, mass, inertia, forces, moments)

            ut[j] = u[j] - du[j]
            fcs = fcs_from_u(trimmed_fcs, ut)
            mass, inertia, forces, moments = recalculate_aircraft(x, trimmed_ac, trimmed_fcs, grav)
            xd2 = six_dof_euler_fixed_mass(x, mass, inertia, forces, moments)
            B[:, j] = (transpose(xd1 - xd2) ./ (2*du[j]))
            if false in (max.(abs.(B[:,j] - last) ./ abs.(B[:,j] .+ 1e-12)) .< tol)
                break
            end
            du[j] = 0.5*du[j]  # reducing du further to improve accuracy of the step
            last = B[:,j]
            iteration = i
            if iteration == 10
                println("not converged on B, column $j")
            end
        end
    end

    B_ = B.*2
    return B_

end


#### utils      => only to be used in this file

"""
returns state::State from the state_array
"""
function state_from_x(x)
    pos = PositionEarth(x[10], x[11], x[12])
    vel = [x[1], x[2], x[3]]
    ang_vel = [x[4], x[5], x[6]]
    att = Attitude(x[7], x[8], x[9])

    return State(pos, att, vel, ang_vel, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
end


"""
calculate Aircraft after modifying states
"""
function recalculate_aircraft(xt::Array, ac, fcs, grav)
    state = state_from_x(xt)
    env = Environment(state.position, atmos="ISAF16", wind="NoWind", grav="const")
    aerostate = AeroState(state, env)
    ac = calculate_aircraft(ac, fcs, aerostate, state, grav)
    mass = ac.mass_props.mass
    inertia = ac.mass_props.inertia
    forces = ac.pfm.forces
    moments = ac.pfm.moments

    return mass, inertia, forces, moments
end

"""
construct a control_array
"""
function get_controls(fcs::FCS)
    u = [fcs.stick_longitudinal.value, fcs.stick_lateral.value, fcs.pedals.value, fcs.thtl.value]

    return u
end

"""
returns fcs with setting controls to modified value as per control_array
"""
function fcs_from_u(fcs, u)
    set_stick_lon(fcs, u[1])
    set_stick_lat(fcs, u[2])
    set_pedals(fcs, u[3])
    set_thtl(fcs, u[4])

    return fcs
end


