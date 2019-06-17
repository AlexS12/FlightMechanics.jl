module Aircrafts
    include("includes.jl")

    include("C310/aerodynamics.jl")
    include("C310/propulsion.jl")
    include("C310/fcs.jl")
    include("C310/C310.jl")

    include("F16/aerodynamics.jl")
    include("F16/propulsion.jl")
    include("F16/fcs.jl")
    include("F16/F16.jl")

    include("trimmer.jl")
end
