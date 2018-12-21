using FlightMechanics
@static if VERSION < v"0.7.0-DEV.2005"
    using Base.Test
else
    using Test
end


@testset "atmosphere" begin include("atmosphere.jl") end
@testset "coordinates" begin include("coordinates.jl") end
@testset "anemometry" begin include("anemometry.jl") end
@testset "mechanics" begin include("mechanics.jl") end

@testset "pfm" begin include("simulator/models/point_forces_moments.jl") end
@testset "mass" begin include("simulator/models/mass.jl") end
@testset "attitude" begin include("simulator/models/attitude.jl") end
@testset "position" begin include("simulator/models/position.jl") end

@testset "c310 func test" begin include("simulator/c310.jl") end
