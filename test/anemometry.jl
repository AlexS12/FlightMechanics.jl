using FlightMechanics


@static if VERSION < v"0.7.0-DEV.2005"
    using Base.Test
else
    using Test
end


# TESTS for TAS CAS EAS based on values from
# http://www.hochwarth.com/misc/AviationCalculator.html

# --- height = 0 m ---
h = 0.0  # m
T, p, rho, a = atmosphere_isa(h)
# From TAS
tas = 100.0 * KT2MS  # m/s
expected_eas = 100.0 * KT2MS  # m/s
expected_cas = 100.0 * KT2MS  # m/s

eas = tas2eas(tas, rho)
cas = tas2cas(tas, rho, p)
@test isapprox(eas, expected_eas)
@test isapprox(cas, expected_cas)

# From CAS
cas = 100.0 * KT2MS  # m/s
expected_eas = 100.0 * KT2MS  # m/s
expected_tas = 100.0 * KT2MS  # m/s

eas = cas2eas(cas, rho, p)
tas = cas2tas(cas, rho, p)
@test isapprox(eas, expected_eas)
@test isapprox(tas, expected_tas)

# From EAS
eas = 100.0 * KT2MS  # m/s
expected_cas = 100.0 * KT2MS  # m/s
expected_tas = 100.0 * KT2MS  # m/s

cas = eas2cas(eas, rho, p)
tas = eas2tas(eas, rho)
@test isapprox(cas, expected_cas)
@test isapprox(tas, expected_tas)


# --- height = 5000 m ---
h = 5000.0  # m
T, p, rho, a = atmosphere_isa(h)
# From TAS
tas = 200.0 * KT2MS  # m/s
expected_eas = 155.03684 * KT2MS  # m/s
expected_cas = 155.95551 * KT2MS  # m/s

eas = tas2eas(tas, rho)
cas = tas2cas(tas, rho, p)
@test isapprox(eas, expected_eas, atol=1e-4)
@test isapprox(cas, expected_cas, atol=1e-4)

# From CAS
cas = 200.0 * KT2MS  # m/s
expected_eas = 198.101308 * KT2MS  # m/s
expected_tas = 255.553851 * KT2MS  # m/s

eas = cas2eas(cas, rho, p)
tas = cas2tas(cas, rho, p)
@test isapprox(eas, expected_eas, atol=1e-4)
@test isapprox(tas, expected_tas, atol=1e-4)

# From EAS
eas = 200.0 * KT2MS  # m/s
expected_cas = 201.95290 * KT2MS  # m/s
expected_tas = 258.00319 * KT2MS  # m/s

cas = eas2cas(eas, rho, p)
tas = eas2tas(eas, rho)
@test isapprox(cas, expected_cas, atol=1e-4)
@test isapprox(tas, expected_tas, atol=1e-4)


# --- Velocities from qc ---
# Values from http://www.aerospaceweb.org/design/scripts/atmosphere/
# geometric altitude = 8010.0807  # m  --> geopotential = 8000.0  # m
# velocity = 100  # m/s
h = 8000.0  # m
T, p, rho, a = atmosphere_isa(h)
tas = 100  # m/s

qinf = 38295.5172  # Pa (Total Head)
qc = qinf - p
exp_cas = 66.0304  # m/s
exp_tas = 100  # m/s
exp_eas = 65.4758  # m/s

eas = qc2eas(qc, p)
tas_ = qc2tas(qc, rho, p)
cas = qc2cas(qc)

@test isapprox(cas, exp_cas, atol=1e-3)
@test isapprox(eas, exp_eas, atol=1e-3)
@test isapprox(tas_, exp_tas, atol=1e-3)


# --- Dynamic pressure ---
# Values from http://www.aerospaceweb.org/design/scripts/atmosphere/
# geometric altitude = 8010.0807  # m  --> geopotential = 8000.0  # m
# velocity = 100  # m/s
h = 8000.0  # m
T, p, rho, a = atmosphere_isa(h)
tas = 100  # m/s

exp_qinf_inc = 2625.8356  # Pa (True Dynamic Pressure)
qinf_inc = incompressible_qinf(tas, rho)
@test isapprox(qinf_inc, exp_qinf_inc, atol=0.0001)

M = tas / a
exp_qinf_comp = 38295.5172  # Pa (Total Head)
qinf_comp = compressible_qinf(M, p)
@test isapprox(qinf_comp, exp_qinf_comp, atol=0.01)


# --- tas, alpha, beta from body ---
u, v, w = 100, 0, 0  # m/s
rv = tas_alpha_beta_from_uvw(u, v, w)
@test isapprox(rv, [100, 0, 0])

u, v, w = 10, 0, 10  # m/s
rv = tas_alpha_beta_from_uvw(u, v, w)
@test isapprox(rv, [sqrt(200), atan(1), 0])

u, v, w = 10, 10, 0  # m/s
rv = tas_alpha_beta_from_uvw(u, v, w)
@test isapprox(rv, [sqrt(200), 0, asin(10/sqrt(200))])
