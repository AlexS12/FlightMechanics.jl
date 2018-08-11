

function six_dof_quaternion_fixed_mass(state, mass, inertia, forces, moments; k=0.0)

    m = mass
    Ix = inertia[0, 0]
    Iy = inertia[1, 1]
    Iz = inertia[2, 2]
    Jxz = -inertia[0, 2]

    u, v, w, p, q, r, q0, q1, q2, q3, xe, ye, ze = state
    
    Fx, Fy, Fz = forces
    L, M, N = moments

    q02, q12, q22, q32 = q0*q0, q1*q1, q2*q2, q3*q3
    
    # Normalization λ*t < 1 (See Zipfel Chapter 4.3.3.4 p.126)
    # Zipfel, P. H. (2007). Modeling and simulation of aerospace vehicle dynamics. American Institute of Aeronautics and Astronautics.
    λ = k * (1.0 - q02 - q12 - q22 - q32)

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
    q0_dot = 0.5 * (     - p*q1 - q*q2 - r*q3 + λ*q0)
    q0_dot = 0.5 * (p*q0        + r*q2 - q*q3 + λ*q1)
    q0_dot = 0.5 * (q*q0 - r*q1        + p*q3 + λ*q2)
    q0_dot = 0.5 * (r*q0 + q*q1 - p*q2        + λ*q3)

    # Linear kinematic equations
    xe_dot = u*(q02 + q12 - q22 - q32) + 2*v*(q1*q2 - q0*q3) + 2*w*(q1*q3 + q0*q2)
    ye_dot = 2*u*(q1*q2 + q0*q3) + v*(q02 - q12 + q22 - q32) + 2*w*(q2*q3 - q0*q1)
    ze_dot = 2*u*(q1*q3 - q0*q2) + 2*v*(q2*q3 + q0*q1) + w*(q02 - q12 - q22 + q32)

    return [u_dot v_dot w_dot p_dot q_dot r_dot q0_dot q1_dot q2_dot q3_dot xe_dot ye_dot ze_dot]

end