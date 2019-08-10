"""

"""
function six_dof_aero_euler_fixed_mass(state, mass, inertia, forces, moments,
                                       h=[0.0, 0.0, 0.0])

    m = mass
    tas, α, β, p, q, r, ψ, θ, ϕ, xe, ye, ze = state

    Fx, Fy, Fz = forces
    L, M, N = moments

    sψ, cψ = sin(ψ), cos(ψ)
    sθ, cθ = sin(θ), cos(θ)
    sϕ, cϕ = sin(ϕ), cos(ϕ)
    sβ, cβ = sin(β), cos(β)
    sα, cα = sin(α), cos(α)

    u = tas * cα * cβ;
    v = v * sβ;
    w = tas * sα * cβ;

    # Linear momentum equations
    u_dot = Fx / m + r * v - q * w
    v_dot = Fy / m - r * u + p * w
    w_dot = Fz / m + q * u - p * v

    tas_dot = (u * u_dot + v * v_dot + w *w_dot) / tas
    β_dot = (tas * v_dot - v * tas) / (cβ * tas*tas)
    α_dot = (w_dot * u - w * u_dot) / (u*u + w*w)

    # Angular momentum equations
    Ix = inertia[1, 1]
    Iy = inertia[2, 2]
    Iz = inertia[3, 3]
    Jxz = -inertia[1, 3]

    Jxz2 = Jxz*Jxz
    Γ = (Ix*Iz - Jxz2)
    temp = (Ix + Iz - Iy)

    # Engine angular momentum contribution
    hx, hy, hz = h

    rhy_qhz = (r*hy - q*hz)
    qhx_phy = (q*hx - p*hy)

    pe_dot = Iz * rhy_qhz + Jxz * qhx_phy
    qe_dot = -r*hx + p*hz
    re_dot = Jxz * rhy_qhz + Ix * qhx_phy

    # Angular momentum equations
    p_dot = L*Iz + N*Jxz - q*r*(Iz*Iz - Iz*Iy + Jxz2) + p*q * Jxz * temp + pe_dot
    p_dot /= Γ
    q_dot = M + (Iz - Ix) * p*r - Jxz * (p*p - r*r) + qe_dot
    q_dot /= Iy
    r_dot = L*Jxz + N*Ix + p*q * (Ix*Ix - Ix*Iy + Jxz2) - q*r * Jxz * temp + re_dot
    r_dot /= Γ

    # Angular Kinematic equations
    ψ_dot = (q * sϕ + r * cϕ) / cθ
    θ_dot = q * cϕ - r * sϕ
    # ϕ_dot = p + (q * sϕ + r * cϕ) * tan(θ)
    ϕ_dot = p + ψ_dot * sθ

    # Linear kinematic equations
    xe_dot =  cθ*cψ * u + (sϕ*sθ*cψ - cϕ*sψ) * v + (cϕ*sθ*cψ + sϕ*sψ) * w
    ye_dot =  cθ*sψ * u + (sϕ*sθ*sψ + cϕ*cψ) * v + (cϕ*sθ*sψ - sϕ*cψ) * w
    ze_dot = -sθ    * u +  sϕ*cθ             * v +  cϕ*cθ             * w

    [tas_dot α_dot β_dot p_dot q_dot r_dot ψ_dot θ_dot ϕ_dot xe_dot ye_dot ze_dot]
end
