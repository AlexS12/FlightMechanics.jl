"""
Attitude objects contains Euler angles (rad) and quaternions.

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


show(io::IO, att::Attitude) = print(
    io,
    "Attitude: ψ=$(rad2deg(att.psi))º, θ=$(rad2deg(att.theta))º, ϕ=$(rad2deg(att.phi))º",
    )


# Constructor from Euler angles
Attitude(psi, theta, phi) = Attitude(psi, theta, phi,
                                     euler2quaternion(psi, theta, phi)...)

# Constructor from quaternions
Attitude(q0, q1, q2, q3) = Attitude(quaternion2euler(q0, q1, q2, q3)...,
                                    q0, q1, q2, q3)

# Getters
"""
    get_euler_angles(att::Attitude)

Return Euler angles ψ, θ, ϕ (yaw, pith, roll).
"""
get_euler_angles(att::Attitude) = [att.psi, att.theta, att.phi]

"""
    get_quaternions(att::Attitude)

Return quaternions. 
"""
get_quaternions(att::Attitude) = [att.q0, att.q1, att.q2, att.q3]


function isapprox(x::Attitude, y::Attitude; rtol=1e-8, atol=0.0, nans=false)

    return isapprox(
        [x.psi, x.theta, x.phi, x.q0, x.q1, x.q2, x.q3],
        [y.psi, y.theta, y.phi, y.q0, y.q1, y.q2, y.q3],
        rtol=rtol, atol=atol, nans=nans,
        )
end
