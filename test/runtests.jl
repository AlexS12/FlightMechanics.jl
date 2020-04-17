using Test
using FlightMechanics


@testset "atmosphere" begin include("atmosphere.jl") end
@testset "coordinates" begin include("coordinates.jl") end
@testset "anemometry" begin include("anemometry.jl") end
@testset "mechanics" begin include("mechanics.jl") end
@testset "flight mechanics" begin include("flight_mechanics.jl") end

@testset "pfm" begin include("models/point_forces_moments.jl") end
@testset "mass" begin include("models/mass.jl") end
@testset "attitude" begin include("models/attitude.jl") end
@testset "position" begin include("models/position.jl") end
@testset "aerodynamics" begin include("models/aerodynamics.jl") end
@testset "aerostate" begin include("models/aero_state.jl") end
@testset "dynamic system" begin include("models/dynamic_system.jl") end
@testset "trimmer" begin include("models/trimmer.jl") end

@testset "ac: c310" begin include("aircrafts/c310.jl") end
@testset "ac: f16" begin include("aircrafts/f16.jl") end
