using FlightMechanics

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
rv_arr = reduce(hcat, rv)
T = rv_arr[1, :]
p = rv_arr[2, :]
rho = rv_arr[3, :]
a = rv_arr[4, :]
@test isapprox(T, [288.150, 287.825, 284.575, 245.900, 223.150, 216.650])
@test isapprox(rho, [1.2250, 1.2191, 1.1616, 0.62384, 0.41271, 0.36392], rtol=1e-4)
@test isapprox(a, [340.29, 340.10, 338.18, 314.36, 299.46, 295.07], rtol=1e-5)

# Test 11-20 Km
h = [12000, 14200, 17500, 20000, ]  # m
rv = atmosphere_isa.(h)
rv_arr = reduce(hcat, rv)
T = rv_arr[1, :]
p = rv_arr[2, :]
rho = rv_arr[3, :]
a = rv_arr[4, :]
@test isapprox(T, [216.650, 216.650, 216.650, 216.650])
@test isapprox(rho, [0.31083, 0.21971, 0.13058, 0.088035], rtol=1e-4)
@test isapprox(a, [295.07, 295.07, 295.07, 295.07], rtol=1e-5)

# Test 20-32 Km
h = [22100, 24000, 28800, 32000]  # m
rv = atmosphere_isa.(h)
rv_arr = reduce(hcat, rv)
T = rv_arr[1, :]
p = rv_arr[2, :]
rho = rv_arr[3, :]
a = rv_arr[4, :]
@test isapprox(T, [218.750, 220.650, 225.450, 228.650])
@test isapprox(rho, [0.062711, 0.046267, 0.021708, 0.013225], rtol=1e-4)
@test isapprox(a, [296.50, 297.78, 301.00, 303.13], rtol=1e-5)

# Test 32-47 Km
h = [32200, 36000, 42000, 47000]  # m
rv = atmosphere_isa.(h)
rv_arr = reduce(hcat, rv)
T = rv_arr[1, :]
p = rv_arr[2, :]
rho = rv_arr[3, :]
a = rv_arr[4, :]
@test isapprox(T, [229.210, 239.850, 256.650, 270.650])
@test isapprox(rho, [0.012805, 0.0070344, 0.0028780, 0.0014275], rtol=1e-4)
@test isapprox(a, [303.50, 310.47, 321.16, 329.80], rtol=1e-5)

# Test 47-51 Km
h = [47200, 49000, 51000]  # m
rv = atmosphere_isa.(h)
rv_arr = reduce(hcat, rv)
T = rv_arr[1, :]
p = rv_arr[2, :]
rho = rv_arr[3, :]
a = rv_arr[4, :]
@test isapprox(T, [270.650, 270.650, 270.650])
@test isapprox(rho, [0.0013919, 0.0011090, 0.00086160], rtol=1e-4)
@test isapprox(a, [329.80, 329.80, 329.80], rtol=1e-5)

# Test 51-71 Km
h = [51500, 60000, 71000]  # m
rv = atmosphere_isa.(h)
rv_arr = reduce(hcat, rv)
T = rv_arr[1, :]
p = rv_arr[2, :]
rho = rv_arr[3, :]
a = rv_arr[4, :]
@test isapprox(T, [269.250, 245.450, 214.650])
@test isapprox(rho, [0.00081298, 2.8832e-4, 6.4211e-5], rtol=1e-4)
@test isapprox(a, [328.94, 314.07, 293.70], rtol=1e-4)

# Test 71-84 Km
h = [52000, 60000, 84500]  # m
rv = atmosphere_isa.(h)
rv_arr = reduce(hcat, rv)
T = rv_arr[1, :]
p = rv_arr[2, :]
rho = rv_arr[3, :]
a = rv_arr[4, :]
@test isapprox(T, [267.850, 245.450, 187.650])
@test isapprox(rho, [7.6687e-4, 2.8832e-4, 7.3914e-6], rtol=1e-4)
@test isapprox(a, [328.09, 314.07, 274.61], rtol=1e-5)

@test_throws DomainError atmosphere_isa(84500.1)
@test_throws DomainError atmosphere_isa(-0.1)
