using Test
using FlightMechanics


@testset "atmosphere" begin include("atmosphere.jl") end
@testset "coordinates" begin include("coordinates.jl") end
@testset "anemometry" begin include("anemometry.jl") end
@testset "mechanics" begin include("mechanics.jl") end
@testset "flight mechanics" begin include("flight_mechanics.jl") end

@testset "pfm" begin include("simulator/models/point_forces_moments.jl") end
@testset "mass" begin include("simulator/models/mass.jl") end
@testset "attitude" begin include("simulator/models/attitude.jl") end
@testset "position" begin include("simulator/models/position.jl") end
@testset "aerodynamics" begin include("simulator/models/aerodynamics.jl") end
@testset "aerostate" begin include("simulator/models/aero_state.jl") end
@testset "trimmer" begin include("simulator/models/trimmer.jl") end

@testset "ac: c310" begin include("simulator/aircrafts/c310.jl") end
@testset "ac: f16" begin include("simulator/aircrafts/f16.jl") end
