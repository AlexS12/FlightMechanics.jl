import FlightMechanics.EarthConstants: ROT_VELOCITY, WGS84
import FlightMechanics: rot_matrix_body2ecef, ecef2llh


ωe = ROT_VELOCITY


"""
.. [1] Stevens, B. L., Lewis, F. L., (1992). Aircraft control and simulation:
 dynamics, controls design, and autonomous systems. John Wiley & Sons.
 (page 45, formula 1.5-1)
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

    state_dot = [vb_dot, ωb_dot, q_dot, p_dot]

    return state_dot
end