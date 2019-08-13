
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


# TODO: generelize d(ψ, θ, ϕ)/dt <=> (p, q, r)
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
    uvw_to_tasαβ(u, v, w)

Calculate true air speed (TAS), angle of attack (α) and angle of side slip (β)
from velocity expressed in body axis.

# Notes
This function assumes that u, v, w are the body components of the aerodynamic
speed. This is not true in genreal (wind speed different from zero), as u, v, w
represent velocity with respect to an inertial reference frame.
"""
function uvw_to_tasαβ(u, v, w)

    tas = sqrt(u*u + v*v + w*w)
    α = atan(w, u)
    β = atan(v, sqrt(u*u + v*v))
    return [tas, α, β]
end
