
function coordinated_turn_bank(turn_rate, alpha, beta, tas, gamma)

    g0 = GRAVITY_ACCEL
    G = turn_rate * tas / g0

    if abs(gamma) < 1e-8
        phi = G * cos(beta) / (cos(alpha) - G * sin(alpha) * sin(beta))
        phi = atan(phi)
    else
        a = 1 - G * tan(alpha) * sin(beta)
        b = sin(gamma) / cos(beta)
        c = 1 + G^2 * cos(beta)^2

        sq = sqrt(c * (1 - b^2) + G^2 * sin(beta)^2)

        num = (a - b^2) + b * tan(alpha) * sq
        den = a ^ 2 - b^2 * (1 + c * tan(alpha)^2)

        phi = atan(G * cos(beta) / cos(alpha) * num / den)
    end
    return phi
end


function climb_theta(gamma, alpha, beta, phi)
    a = cos(alpha) * cos(beta)
    b = sin(phi) * sin(beta) + cos(phi) * sin(alpha) * cos(beta)
    sq = sqrt(a^2 - sin(gamma)^2 + b^2)
    theta = (a * b + sin(gamma) * sq) / (a^2 - sin(gamma)^2)
    theta = atan(theta)
    return theta
end


function turn_rate_angular_velocity(turn_rate, theta, phi)
    # w = turn_rate * k_h
    # k_h = sin(theta) i_b + sin(phi) * cos(theta) j_b + cos(theta) * sin(phi)
    # w = p * i_b + q * j_b + r * k_b
    p = - turn_rate * sin(theta)
    q = turn_rate * sin(phi) * cos(theta)
    r = turn_rate * cos(theta) * cos(phi)
    return [p, q, r]
end


"""
    body_angular_velocity_to_euler_angles_rates(p, q, r, ψ, θ, ϕ)

Transform body angular velocity (p, q, r) [rad/s] to Euler angles rates
(ψ_dot, θ_dot, ϕ_dot) [rad/s] given the euler angles (θ, ϕ) [rad] using
kinematic angular equations.

# References

- [1] Stevens, B. L., Lewis, F. L., & Johnson, E. N. (2015). Aircraft control
 and simulation: dynamics, controls design, and autonomous systems. John Wiley
 & Sons. Equation (1.4-4) (page 20)
"""
function body_angular_velocity_to_euler_angles_rates(p, q, r, θ, ϕ)

    sθ, cθ = sin(θ), cos(θ)
    sϕ, cϕ = sin(ϕ), cos(ϕ)

    ψ_dot = (q * sϕ + r * cϕ) / cθ
    θ_dot = q * cϕ - r * sϕ
    # ϕ_dot = p + (q * sϕ + r * cϕ) * tan(θ)
    ϕ_dot = p + ψ_dot * sθ

    return [ψ_dot, θ_dot, ϕ_dot]
end


"""
    euler_angles_rates_to_body_angular_velocity(ψ_dot, θ_dot, ϕ_dot, ψ, θ, ϕ)

Transform Euler angles rates (ψ_dot, θ_dot, ϕ_dot) [rad/s] to body angular
velocity (p, q, r) [rad/s] given the euler angles (θ, ϕ) [rad] using
kinematic angular equations.

# References

- [1] Stevens, B. L., Lewis, F. L., & Johnson, E. N. (2015). Aircraft control
 and simulation: dynamics, controls design, and autonomous systems. John Wiley
 & Sons. Equation (1.4-3) (page 20)
"""
function euler_angles_rates_to_body_angular_velocity(ψ_dot, θ_dot, ϕ_dot, θ, ϕ)

    sθ, cθ = sin(θ), cos(θ)
    sϕ, cϕ = sin(ϕ), cos(ϕ)

    p = ϕ_dot - sθ * ψ_dot
    q = cϕ * θ_dot + sϕ*cθ * ψ_dot
    r = -sϕ * θ_dot + cϕ*cθ * ψ_dot

    return [p, q, r]
end


"""
    body_angular_velocity_to_quaternion_rates(p, q, r, q0, q1, q2, q3)

Transform body angular velocity (p, q, r) [rad/s] to quaternion rates [1/s].

- [1] Stevens, B. L., Lewis, F. L., & Johnson, E. N. (2015). Aircraft control
 and simulation: dynamics, controls design, and autonomous systems. John Wiley
 & Sons. Equation (1.8-15) (page 51)

"""
function body_angular_velocity_to_quaternion_rates(p, q, r, q0, q1, q2, q3)

    Ω = [
        0 -p -q -r;
        p  0  r -q;
        q -r  0  p;
        r  q -p  0;
    ]
    q = [q0; q1; q2; q3]
    q_dot = 0.5 * Ω * q

    return q_dot
end


"""
    uvw_to_tasαβ(u, v, w)

Calculate true air speed (TAS), angle of attack (α) and angle of side slip (β)
from velocity expressed in body axis.

# Notes
This function assumes that u, v, w are the body components of the aerodynamic
speed. This is not true in genreal (wind speed different from zero), as u, v, w
represent velocity with respect to an inertial reference frame.

# References

- [1] Stevens, B. L., Lewis, F. L., & Johnson, E. N. (2015). Aircraft control
 and simulation: dynamics, controls design, and autonomous systems. John Wiley
 & Sons. Equation (2.3-6b) (page 78)
"""
function uvw_to_tasαβ(u, v, w)

    tas = sqrt(u*u + v*v + w*w)
    α = atan(w, u)
    β = asin(v / tas)
    return [tas, α, β]
end


"""
    tas_α_β_dot_from_uvw_dot(u, v, w, u_dot, v_dot, w_dot)

Calculate time derivatives of velcity expressed as TAS, AOA, AOS.

# Notes
Note that tas here is not necessarily true air speed. Could also be inertial speed in the
direction of airspeed. It will concide whith TAS for no wind.

# See also
    uvw_dot_from_tas_α_β(tas, α, β, tas_dot, α_dot, β_dot)

# References
- [1] Morelli, Eugene A., and Vladislav Klein. Aircraft system identification: Theory and
 practice. Williamsburg, VA: Sunflyte Enterprises, 2016. Equation 3.33 (page 44).
- [2] Stevens, B. L., Lewis, F. L., & Johnson, E. N. (2015). Aircraft control
 and simulation: dynamics, controls design, and autonomous systems. John Wiley
 & Sons. Equation (2.3-10) (page 81)
"""
function tas_α_β_dot_from_uvw_dot(u, v, w, u_dot, v_dot, w_dot)

    # [1] 3.31
    tas = sqrt(u*u + v*v + w*w)
    # [1] 3.33a
    tas_dot = (u * u_dot + v * v_dot + w * w_dot) / tas
    # [1] 3.33b
    β_dot = ((u*u + w*w) * v_dot - v * (u * u_dot + w * w_dot)) / (tas^2 * sqrt(u*u + w*w))
    # [2] 2.3.10b
    # β_dot = (v_dot * tas - v * tas_dot) / (tas * sqrt(u*u + w*w))
    # [1] 3.33c
    α_dot = (w_dot * u - w * u_dot) / (u*u + w*w)

    return [tas_dot, α_dot, β_dot]
end


"""
    uvw_dot_from_tas_α_β(tas, α, β, tas_dot, α_dot, β_dot)

Obatain body velocity derivatives given velocity in wind axis and its derivatives.

# See also
    tas_α_β_dot_from_uvw_dot(u, v, w, u_dot, v_dot, w_dot)

# References
- [1] Morelli, Eugene A., and Vladislav Klein. Aircraft system identification: Theory and
 practice. Williamsburg, VA: Sunflyte Enterprises, 2016. Derived from equation 3.32 (page 44).
"""
function uvw_dot_from_tas_α_β(tas, α, β, tas_dot, α_dot, β_dot)
    u_dot = tas_dot * cos(α) * cos(β) - tas * (α_dot * sin(α) * cos(β) + β_dot * cos(α) * sin(β))
    v_dot = tas_dot * sin(β) + tas * β_dot * cos(β)
    w_dot = tas_dot * sin(α) * cos(β) + tas * (α_dot * cos(α) * cos(β) - β_dot * sin(α) * sin(β))
    return [u_dot, v_dot, w_dot]
end
