import FlightMechanics: atmosphere_isa, tas2eas, tas2cas, cas2eas, cas2tas, eas2cas, eas2tas
import FlightMechanics.ConversionConstants: KT2MS

@static if VERSION < v"0.7.0-DEV.2005"
    using Base.Test
else
    using Test
end


# TESTS based on values from
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
