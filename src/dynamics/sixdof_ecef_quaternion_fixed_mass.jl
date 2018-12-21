using FlightMechanics

export six_dof_ecef_quaternion_fixed_mass

const ωe = ROT_VELOCITY

"""
    six_dof_ecef_quaternion_fixed_mass(state, mass, inertia, forces, moments;
    k=0.0, ellipsoid=WGS84)

Six degrees of freedom dynamic system using quaternions for attitude
representation and assuming fixed mass.

Ellipsoidal Earth Model is used and the ECEF reference frame is considered
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
- `ellipsoid::Ellipsoid`: ellipsoid model to be used.

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
- Implementation based on [1]. However, [2] can also be read.

# References
- [1] Stevens, B. L., Lewis, F. L., (1992). Aircraft control and simulation:
 dynamics, controls design, and autonomous systems. John Wiley & Sons.
 (page 45, formula 1.5-1)

- [2] Zipfel, P. H. (2007). Modeling and simulation of aerospace vehicle
 dynamics. American Institute of Aeronautics and Astronautics.
 (page 396, figure 10.6)
"""
function six_dof_ecef_quaternion_fixed_mass(state, mass, inertia, forces, moments;
     k=0.0, ellipsoid=WGS84)

    vb = state[1:3]    # u, v, w
    ωb = state[4:6]    # p, q, r
    q  = state[7:10]   # q0, q1, q2, q3
    p  = state[11:13]  # px, py, pz (ecef)

    Fb = forces
    J  = inertia
    Tb = moments

    Ωe = [0.0   ωe    0.0;
          -ωe   0.0   0.0;
          0.0   0.0   0.0]

    p, q, r = ωb

    ΩB = [0.0   -r   -q;
           r    0.0  -p;
          -q     p   0.0]

    Ωq = [0.0    p    q    r;
          -p    0.0  -r    q;
          -q     r   0.0  -p;
          -r    -q    p   0.0]

    lat, lon, h = ecef2llh(p...; ellipsoid=ellipsoid)
    B = rot_matrix_body2ecef(lat, lon, q...)

    # Linear kinematic equations
    p_dot = Ωe * p + B' * vb
    # Linear momentum equations
    # Note that gravity (B g(p) - B Ωe²) p is included in forces (gravity + prop + aero)
    vb_dot = -(ΩB + B*Ωe) * vb + Fb / mass
    # Angular momentum equations
    Jinv = inv(J)
    ωb_dot = (-Jinv * ΩB * J) * ωb + Jinv * Tb
    # Angular Kinematic equations
    # Normalization λ*t < 1 (See Zipfel Chapter 4.3.3.4 p.126)
    # Zipfel, P. H. (2007). Modeling and simulation of aerospace vehicle dynamics.
    # American Institute of Aeronautics and Astronautics.
    λ = k * (1.0 - q*q)
    q_dot = (-0.5 * Ωq + λ) * q

    state_dot = [vb_dot..., ωb_dot..., q_dot..., p_dot...]

    return state_dot
end
