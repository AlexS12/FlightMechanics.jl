using Test
using LinearAlgebra
using FlightMechanics
using FlightMechanics.Aircrafts
using FlightMechanics.Models


# Integration test for aircraft just making sure everything runs
ac = F16()
fcs = F16FCS()

set_stick_lon!(fcs, 0.0)
set_stick_lat!(fcs, 0.5)
set_pedals!(fcs, 0.5)
set_thtl!(fcs, 0.8)

h = 0.0 * M2FT
psi = 0.0  # rad
gamma = 0.0
turn_rate = 0.0

pos = PositionEarth(0, 0, -h)
env = Environment(pos, atmos = "ISAF16", wind = "NoWind", grav = "const")

#  Stevens, B. L., Lewis, F. L., & Johnson, E. N. (2015). Aircraft control
#  and simulation: dynamics, controls design, and autonomous systems. John Wiley
#  & Sons. (page 193 table 3.6-2)
tas =  500 * FT2M
exp_thtl = 0.137
exp_α    = 2.14
exp_de   = -0.756

# Initial conditions for trimmer
α0 = exp_α * DEG2RAD + 5 * DEG2RAD
β0 = 0.
stick_lon0 = exp_de / 25.0 + 0.5
thtl0 = exp_thtl + 0.3

set_stick_lon!(fcs, stick_lon0)
set_thtl!(fcs, thtl0)

fcs_before_trimming = copy(fcs)

ac_trim, aerostate_trim, state_trim, fcs_trim = steady_state_trim(
    ac, fcs, env, tas, pos, psi, gamma, turn_rate, α0, 0.0,
    )

# TEST: FCS has not been modified by trimmer
@test isapprox(get_controls_trimmer(fcs), get_controls_trimmer(fcs_before_trimming))

# TEST: Check results against Stevens (also checked in f16.jl tests)
@test isapprox(ac_trim.pfm.forces, zeros(3), atol = 1e-7)
@test isapprox(ac_trim.pfm.moments, zeros(3), atol = 1e-7)
@test isapprox(get_tas(aerostate_trim), tas)
@test isapprox(get_beta(aerostate_trim), 0.0, atol = 1e-13)

@test isapprox(fcs_trim.thtl.value, exp_thtl, atol = 0.001)
@test isapprox(aerostate_trim.alpha * RAD2DEG, exp_α, atol = 0.01)
@test isapprox(fcs_trim.de.value * RAD2DEG, exp_de, atol = 0.001)

# TEST: calculate aircraft on trimmed aricraft and check that forces and moments
# are the same.
ac_calc = calculate_aircraft(
    ac_trim,
    fcs_trim,
    aerostate_trim,
    state_trim,
    get_gravity(env),
    )
@test isapprox(ac_trim.pfm.forces, ac_calc.pfm.forces)
@test isapprox(ac_trim.pfm.moments, ac_calc.pfm.moments)

# TEST: Trim a trimmed aircraft and check if returns the same trim
ac_trim2, aerostate_trim2, state_trim2, fcs_trim2 = steady_state_trim(
    ac_trim,
    fcs_trim,
    env,
    tas,
    pos,
    psi,
    gamma,
    turn_rate,
    get_alpha(aerostate_trim),
    get_beta(aerostate_trim),
    )

@test isapprox(get_tas(aerostate_trim2), tas)
@test isapprox(get_beta(aerostate_trim2),  0.0, atol = 1e-13)

@test isapprox(fcs_trim2.thtl.value, exp_thtl, atol = 0.001)
@test isapprox(aerostate_trim2.alpha * RAD2DEG, exp_α, atol = 0.01)
@test isapprox(fcs_trim2.de.value * RAD2DEG, exp_de, atol = 0.001)

@test isapprox(ac_trim2.pfm.forces, ac_trim.pfm.forces)
@test isapprox(ac_trim2.pfm.moments, ac_trim.pfm.moments)
