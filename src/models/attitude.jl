using FlightMechanics

export Attitude,
       get_euler_angles,
       get_quaternions


"""
Attitude objects that contains Euler angles and quaternions.

# Constructors
    Attitude(psi, theta, phi)
    Attitude(q0, q1, q2, q3)
"""
struct Attitude
    # Euler angles
    psi::Number
    theta::Number
    phi::Number
    # Quaternions
    q0::Number
    q1::Number
    q2::Number
    q3::Number
end

# Constructor from Euler angles
Attitude(psi, theta, phi) = Attitude(psi, theta, phi,
                                     euler2quaternion(psi, theta, phi)...)

# Constructor from quaternions
Attitude(q0, q1, q2, q3) = Attitude(quaternion2euler(q0, q1, q2, q3)...,
                                    q0, q1, q2, q3)

# Getters
get_euler_angles(att::Attitude) = [att.psi, att.theta, att.phi]
get_quaternions(att::Attitude) = [att.q0, att.q1, att.q2, att.q3]
