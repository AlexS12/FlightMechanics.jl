import FlightMechanics: atmosphere_isa

@static if VERSION < v"0.7.0-DEV.2005"
    using Base.Test
else
    using Test
end


# Test sea level
T, p, rho, a = atmosphere_isa(0.0)
@test isapprox(T, 288.15)
@test isapprox(p, 101325)
@test isapprox(rho, 1.225)
@test isapprox(a, 340.29398, rtol=1e-7)

# Test 0-11 Km
h = [0.0, 50.0, 550.0, 6500.0, 10000.0, 11000.0]
# TODO: have a look again at unzip options
# I would expect a tuple of Arrays, however an Array of tuples is returned.
# https://discourse.julialang.org/t/broadcast-map-return-type-for-a-function-returning-a-tuple/574/6
rv = atmosphere_isa.(h)
rv_arr = reinterpret(Float64, rv, (4, size(h, 1)))
T = rv_arr[1, :]
p = rv_arr[2, :]
rho = rv_arr[3, :]
a = rv_arr[4, :]
@test isapprox(T, [288.150, 287.825, 284.575, 245.900, 223.150, 216.650])
@test isapprox(rho, [1.2250, 1.2191, 1.1616, 0.62384, 0.41271, 0.36392], rtol=1e-4)
@test isapprox(a, [340.29, 340.10, 338.18, 314.36, 299.46, 295.07], rtol=1e-5)