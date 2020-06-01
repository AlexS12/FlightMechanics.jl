using Test
using FlightMechanics


ψ, θ, ϕ = 0.5, 0.05, 0.2  # rad
p, q, r = 1.0, 0.2, 0.1  # rad/s

# Test regression
ear = body_angular_velocity_to_euler_angles_rates(p, q, r, θ, ϕ)
pqr = euler_angles_rates_to_body_angular_velocity(ear..., θ, ϕ)
@test isapprox(pqr, [p, q, r])

# Test against trimmer function
ψ_dot = 0.2
pqr1 = euler_angles_rates_to_body_angular_velocity(ψ_dot, 0.0, 0.0, θ, ϕ)
pqr2 = turn_rate_angular_velocity(ψ_dot, θ, ϕ)
@test isapprox(pqr1, pqr2)

# Test body_angular_velocity_to_quaternion_rates
quat = euler2quaternion(ψ, θ, ϕ)
qdot = body_angular_velocity_to_quaternion_rates(p, q, r, quat...)

Ω_inv = inv(
    [0 -p -q -r;
     p  0  r -q;
     q -r  0  p;
     r  q -p  0]
)
@test isapprox(Ω_inv * qdot * 2, quat)

# Test uvw_to_tasαβ
exp_tas = 100  # m/s
exp_α = 0.01  # rad
exp_β = 0.005  # rad
u, v, w = wind2body(exp_tas, 0, 0, exp_α, exp_β)
tas, α, β = uvw_to_tasαβ(u, v, w)
@test isapprox(tas, exp_tas)
@test isapprox(α, exp_α)
@test isapprox(β, exp_β)


# Test tas_α_β_dot_from_uvw_dot
u = 50.0
v = 1.0
w = 5.0
u_dot = 5.0
v_dot = 0.5
w_dot = 2.0
tas_dot, α_dot, β_dot = tas_α_β_dot_from_uvw_dot(u, v, w, u_dot, v_dot, w_dot)

# Test alpha_dot against formula and tas_dot
tas = √(u*u + v*v + w*w)
@test isapprox(α_dot, (w_dot * u - w * u_dot) / (u*u + w*w))
@test isapprox(tas_dot, (u * u_dot + v * v_dot + w * w_dot) / tas)
# Test β_dot against Stevens implementation (function uses Morelli)
@test isapprox(β_dot, (v_dot * tas - v * tas_dot) / (tas * sqrt(u*u + w*w)))

# tas_α_β_dot_from_uvw_dot <--> uvw_dot_from_tas_α_β
tas, α, β = uvw_to_tasαβ(u, v, w)
u_dot_, v_dot_, w_dot_ = uvw_dot_from_tas_α_β(tas, α, β, tas_dot, α_dot, β_dot)
@test isapprox([u_dot, v_dot, w_dot], [u_dot_, v_dot_, w_dot_])
