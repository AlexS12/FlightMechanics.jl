using FlightMechanics
@static if VERSION < v"0.7.0-DEV.2005"
    using Base.Test
else
    using Test
end

# write your own tests here
@testset "atmosphere" begin include("./test/atmosphere.jl") end