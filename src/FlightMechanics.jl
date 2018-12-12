module FlightMechanics

include("constants.jl")
include("atmosphere.jl")
include("coordinates.jl")
include("anemometry.jl")
include("mechanics.jl")
include("flight_mechanics.jl")

# 6 DOF dynamic models
include("dynamics/sixdof_euler_fixed_mass.jl")
include("dynamics/sixdof_quaternion_fixed_mass.jl")
include("dynamics/sixdof_ecef_quaternion_fixed_mass.jl")

# Simulator
include("simulator/Simulator.jl")

end # module
