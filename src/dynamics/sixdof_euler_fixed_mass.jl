

function six_dof_euler_fixed_mass(state, mass, inertia, forces, moments)

    m = mass
    Ix = inertia[0, 0]
    Iy = inertia[1, 1]
    Iz = inertia[2, 2]
    Jxz = -inertia[0, 2]

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
    p_dot = (L*Iz + N*Jxz - q*r*(Iz*Iz - Iz*Iy + Jxz*Jxz) + p*q * Jxz*(Ix + Iz - Iy)) / (Ix*Iz - Jxz*Jxz)
    q_dot = (M + (Iz - Ix) * p*r - Jxz * (p*p - r*r)) / Iy
    r_dot = (L*Jxz + N*Ix + p*q * (Ix*Ix - Ix*Iy + Jxz*Jxz) - q*r * Jxz * (Iz + Ix - Iy)) / (Ix*Iz - Jxz*Jxz)

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