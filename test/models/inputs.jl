using FlightMechanics.Models

# Constant input
ci_one = ConstantInput(1.0)
ci_minus_one = ConstantInput(-1.0)

@test isapprox(get_value(ci_one, 10.), 1.0)
@test isapprox(get_value(ci_minus_one, 10.), -1.0)
@test isapprox(get_value(vcat(ci_one, ci_minus_one), 10.), 0.0, atol=1e-15)

# StepInput
si1 = StepInput(1.0, 5.0, 10.0)
si2 = StepInput(-1.0, 10.0, 15.0)

@test isapprox(get_value(si1, 0.0), 0.0)
@test isapprox(get_value(si2, 0.0), 0.0)
@test isapprox(get_value(si1, 5.0), 1.0)
@test isapprox(get_value(si1, 10.0), 0.0)
@test isapprox(get_value(si2, 10.0), -1.0)
@test isapprox(get_value(si2, 15.0), 0.0)

@test isapprox(get_value(vcat(si1, si2), 8), 1.0)
@test isapprox(get_value(vcat(si1, si2), 10), -1.0)
@test isapprox(get_value(vcat(si1, si2), 12), -1.0)

# Doublet input
d = DoubletInput(2, 0, 10)
id = InverseDoubletInput(2, 0, 10)

@test isapprox(get_value(d, 2.5), 1)
@test isapprox(get_value(d, 5), -1)
@test isapprox(get_value(d, 7.5), -1)
@test isapprox(get_value(d, 10.), 0)

@test isapprox(get_value(id, 2.5), -1)
@test isapprox(get_value(id, 5), 1)
@test isapprox(get_value(id, 7.5), 1)
@test isapprox(get_value(id, 10.), 0)

@test isapprox(get_value(vcat(d, id), 2.5), 0)
@test isapprox(get_value(vcat(d, id), 5), 0)
@test isapprox(get_value(vcat(d, id), 7.5), 0)
@test isapprox(get_value(vcat(d, id), 10.), 0)

# Ramp input
r1 = RampInput(10, 0, 10)
r2 = RampInput(-10, 10, 15)

@test isapprox(get_value(r1, 0.0), 0.0)
@test isapprox(get_value(r1, 10.0), 0.0)
@test isapprox(get_value(r1, 5.0), 5.0)
@test isapprox(get_value(r2, 10.0), 0.0)
@test isapprox(get_value(r2, 15.0), 0.0)
@test isapprox(get_value(r2, 12.5), -5.0)

@test isapprox(get_value(vcat(r1, r2, StepInput(10, 10, 15)), 10), 10.0)
@test isapprox(get_value(vcat(r1, r2, StepInput(10, 10, 15)), 5), 5.0)
@test isapprox(get_value(vcat(r1, r2, StepInput(10, 10, 15)), 12.5), 5.0)


# Sinusoidal input
s = SinusoidalInput(2, 1, 0, 0, Inf)

@test isapprox(get_value(s, 0), 0)
@test isapprox(get_value(s, 1), 0, atol=1e-15)
@test isapprox(get_value(s, 0.5), 0, atol=1e-15)
@test isapprox(get_value(s, 0.25), 1)
@test isapprox(get_value(s, 0.75), -1)

sA = SinusoidalInput(20, 1, 0, 0, Inf)

@test isapprox(get_value(sA, 0.25), 10)
@test isapprox(get_value(sA, 0.75), -10)

sf = SinusoidalInput(2, 2, 0, 0, Inf)

@test isapprox(get_value(sf, 0.5), 0, atol=1e-15)
@test isapprox(get_value(sf, 0.25), 0, atol=1e-15)

sϕ = SinusoidalInput(2, 1, π/2, 0, Inf)

@test isapprox(get_value(sϕ, 0), 1.0)
@test isapprox(get_value(sϕ, 1), 1.0, atol=1e-15)
