export six_dof_quaternion_fixed_mass

"""
    six_dof_quaternion_fixed_mass(state, mass, inertia, forces, moments; k=0.0)

Six degrees of freedom dynamic system using quaternions for attitude
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
    q0, q1, q2, q3: attitude given by quaternions.
    xe, ye, ze: position wrt the inertial system origin expressed in Earth Axis. (m)
- `mass::Number`: total mass of the aircraft (kg)
- `inertia::3×3 Array{Number,2}`: inertia tensor (kg·m²)
- `forces::3-element Array{Number,1}`: total forces expressed in body axis. (N)
- `moments::3-element Array{Number,1}`: total moments expressed in body axis.(N·m)
- `k::Number`: orthonormality error factor.

# Returns
- `state_dot`: state vector derivative according to the equation of motion,
    inertial properties and applied forces and moments.

# Notes
- See [1] (page 41, formula 1.4-23) or [2] (page 372, formulas 10-11 to 10-14)
  for more information on quaternions <-> Euler angles conversions.
- Orthonormality error factor is related to a numerical stability artifact used
  in angular kinematic equations. Let λ = k * (1 - q0² - q1² - q2² - q3²) be
  the orthonormality error. The term k·λ·q is added to the angular kinematic
  equations in order to reduce the numerical integration error. According to
  reference [2] k·Δt ≤ 1. See [2] (page 372) for more information on
  orthonormality error factor.

# References
- [1] Stevens, B. L., Lewis, F. L., (1992). Aircraft control and simulation:
 dynamics, controls design, and autonomous systems. John Wiley & Sons.
 (Section 1.5, equations 1.5-4, page 46)

- [2] Zipfel, P. H. (2007). Modeling and simulation of aerospace vehicle
 dynamics. American Institute of Aeronautics and Astronautics.
 (page 368, figure 10.2)
"""
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
    # Zipfel, P. H. (2007). Modeling and simulation of aerospace vehicle dynamics.
    # American Institute of Aeronautics and Astronautics.
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
    q0_dot = 0.5 * (     - p*q1 - q*q2 - r*q3 ) + λ * q0
    q1_dot = 0.5 * (p*q0        + r*q2 - q*q3 ) + λ * q1
    q2_dot = 0.5 * (q*q0 - r*q1        + p*q3 ) + λ * q2
    q3_dot = 0.5 * (r*q0 + q*q1 - p*q2        ) + λ * q3

    # Linear kinematic equations
    xe_dot = u*(q02 + q12 - q22 - q32) + 2*v*(q1*q2 - q0*q3) + 2*w*(q1*q3 + q0*q2)
    ye_dot = 2*u*(q1*q2 + q0*q3) + v*(q02 - q12 + q22 - q32) + 2*w*(q2*q3 - q0*q1)
    ze_dot = 2*u*(q1*q3 - q0*q2) + 2*v*(q2*q3 + q0*q1) + w*(q02 - q12 - q22 + q32)

    return [u_dot v_dot w_dot p_dot q_dot r_dot q0_dot q1_dot q2_dot q3_dot xe_dot ye_dot ze_dot]

end
