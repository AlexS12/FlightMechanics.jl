using FlightMechanics

export Attitude,
       get_euler_angles,
       get_quaternions

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
function Attitude(psi, theta, phi)
    Attitude(psi, theta, phi, euler2quaternion(psi, theta, phi)...)
end

# Constructor from quaternions
function Attitude(q0, q1, q2, q3)
    Attitude(quaternion2euler(q0, q1, q2, q3)..., q0, q1, q2, q3)
end


function get_euler_angles(att::Attitude)
    return [att.psi, att.theta, att.phi]
end


function get_quaternions(att::Attitude)
    return [att.q0, att.q1, att.q2, att.q3]
end
