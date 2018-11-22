
export six_dof_euler_fixed_mass

"""
    six_dof_euler_fixed_mass(state, mass, inertia, forces, moments)

Six degrees of freedom dynamic system using Euler angles for attitude
representation and assuming fixed mass.

Flat Earth hypothesis is applied and Earth reference frame is considered
inertial.

It is considered that the aircraft xb-zb plane is a plane of symmetry so that
Jxy and Jyz cross-product of inertia are zero and will not be taken into
account.

# Arguments
- `state::12-element Array{Number,1}`: state vector.
    u, v, w: inertial linear velocity expressed in body axis. (m/s)
    p, q, r: inertial rotatinal velocity expressed in body axis. (rad/s)
    ψ, θ, ϕ: attitude given by Euler angles (zyx). Yaw, pitch, roll. (rad)
    xe, ye, ze: position wrt the inertial system origin expressed in Earth Axis. (m)
- `mass::Number`: total mass of the aircraft (kg)
- `inertia::3×3 Array{Number,2}`: inertia tensor (kg·m²)
- `forces::3-element Array{Number,1}`: total forces expressed in body axis. (N)
- `moments::3-element Array{Number,1}`: total moments expressed in body axis.(N·m)

# Returns
- `state_dot`: state vector derivative according to the equation of motion,
    inertial properties and applied forces and moments.

# References
- [1] Stevens, B. L., Lewis, F. L., (1992). Aircraft control and simulation:
 dynamics, controls design, and autonomous systems. John Wiley & Sons.
 (Section 1.5, equations 1.5-4, page 46)

- [2] Etkin, B. (2005). Dynamics of atmospheric flight. Dover Publications
 (Section 5.8, page 148, formulas 5.8,1 to 5.8,7)

- [3] Zipfel, P. H. (2007). Modeling and simulation of aerospace vehicle
 dynamics. American Institute of Aeronautics and Astronautics.
 (page 368, figure 10.2, not taking into account quaternions in angular
 kinematic equations)
"""
function six_dof_euler_fixed_mass(state, mass, inertia, forces, moments)

    m = mass
    Ix = inertia[1, 1]
    Iy = inertia[2, 2]
    Iz = inertia[3, 3]
    Jxz = -inertia[1, 3]

    u, v, w, p, q, r, ψ, θ, ϕ, xe, ye, ze = state

    Fx, Fy, Fz = forces
    L, M, N = moments

    sψ, cψ = sin(ψ), cos(ψ)
    sθ, cθ = sin(θ), cos(θ)
    sϕ, cϕ = sin(ϕ), cos(ϕ)

    # Linear momentum equations
    u_dot = Fx / m + r * v - q * w
    v_dot = Fy / m - r * u + p * w
    w_dot = Fz / m + q * u - p * v

    # Angular momentum equations
    Jxz2 = Jxz*Jxz
    den = (Ix*Iz - Jxz2)
    temp = (Ix + Iz - Iy)

    p_dot = (L*Iz + N*Jxz - q*r*(Iz*Iz - Iz*Iy + Jxz2) + p*q * Jxz * temp) / den
    q_dot = (M + (Iz - Ix) * p*r - Jxz * (p*p - r*r)) / Iy
    r_dot = (L*Jxz + N*Ix + p*q * (Ix*Ix - Ix*Iy + Jxz2) - q*r * Jxz * temp) / den

    # Angular Kinematic equations
    ψ_dot = (q * sϕ + r * cϕ) / cθ
    θ_dot = q * cϕ - r * sϕ
    ϕ_dot = p + (q * sϕ + r * cϕ) * tan(θ)

    # Linear kinematic equations
    xe_dot =  cθ*cψ * u + (sϕ*sθ*cψ - cϕ*sψ) * v + (cϕ*sθ*cψ + sϕ*sψ) * w
    ye_dot =  cθ*sψ * u + (sϕ*sθ*sψ + cϕ*cψ) * v + (cϕ*sθ*sψ - sϕ*cψ) * w
    ze_dot = -sθ    * u +  sϕ*cθ             * v +  cϕ*cθ             * w

    return [u_dot v_dot w_dot p_dot q_dot r_dot ψ_dot θ_dot ϕ_dot xe_dot ye_dot ze_dot]

end
