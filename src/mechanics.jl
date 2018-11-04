using LinearAlgebra
using Markdown


export rigid_body_velocity, rigid_body_acceleration,
    steiner_inertia

# KINEMATICS
@doc doc"""
    rigid_body_velocity(vel_P, ω, r_PQ)

Calculate the velocity of a point Q of a rigid solid given the velocity of a
point P (vel_P), the rotational velocity of the solid (ω) and the relative
position of Q wrt P.

If the reference frame 1 is attached to the solid and the velocity is
calculated with respect to reference frame 0:

``v_{10}^{Q} = v_{10}^{P} + \omega_{10} \times r^{PQ}``

being:
- ``v_{10}^{Q}`` the velocity of point Q, fixed to 1, wrt 0
- ``\omega_{10}`` the angular velocity of the solid 1 wrt 0
- ``r^{PQ}`` the position of Q wrt P (``r^{Q}-r^{P}``)

Every vector needs to be expressed in the same coordinate system.

# References
- [1] Stevens, B. L., Lewis, F. L., (1992). Aircraft control and simulation:
 dynamics, controls design, and autonomous systems. John Wiley & Sons.
 (Section 1.3, page 26)
"""
function rigid_body_velocity(vel_P, ω, r_PQ)
    vel_Q = vel_P + cross(ω, r_PQ)
    return vel_Q
end


@doc doc"""
    rigid_body_acceleration(acc_P, ω, ω_dot, r_PQ)

Calculate the acceleration of a point Q of a rigid solid given the acceleration
of a point P (acc_P), the rotational velocity of the solid (ω), the rotational
acceleration of the solid (ω_dot) and the relative position of Q wrt P.

``a_{10}^{Q} = a_{10}^{P} + \omega_{10} \times (\omega_{10} \times r^{PQ}) + \dot{\omega}_{10} \times r^{PQ}``

being:
- ``a_{10}^{Q}`` the acceleration of point Q, fixed to 1, wrt 0
- ``\omega_{10}`` the angular velocity of the solid 1 wrt 0
- ``\dot{\omega}_{10}`` the angular acceleration of the solid 1 wrt 0
- ``r^{PQ}`` the position of Q wrt P (``r^{Q}-r^{P}``)

# References
- [1] Stevens, B. L., Lewis, F. L., (1992). Aircraft control and simulation:
 dynamics, controls design, and autonomous systems. John Wiley & Sons.
 (Section 1.3, Formaula 1.3-14c, page 26)
"""
function rigid_body_acceleration(acc_P, ω, ω_dot, r_PQ)
    acc_Q = acc_P + cross(ω, cross(ω, r_PQ)) + cross(ω_dot, r_PQ)
    return acc_Q
end


# Mechanics
function steiner_inertia(point0, inertia0, mass, point1)
    r = point1 - point0
    inertia0 + mass * (dot(r, r) * I - r*r')
end
