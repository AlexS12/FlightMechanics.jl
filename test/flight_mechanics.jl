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

# Test uvw_to_tasαβ
exp_tas = 100  # m/s
exp_α = 0.01  # rad
exp_β = 0.005  # rad
u, v, w = wind2body(tas, 0, 0, exp_α, exp_β)
tas, α, β = uvw_to_tasαβ(u, v, w)
@test isapprox(tas, exp_tas)
@test isapprox(α, exp_α)
@test isapprox(β, exp_β, rtol=1e-3)
