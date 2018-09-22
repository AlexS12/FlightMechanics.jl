module FlightMechanics

include("constants.jl")
include("atmosphere.jl")
include("coordinates.jl")
include("anemometry.jl")

# 6 DOF dynamic models
include("dynamics/sixdof_euler_fixed_mass.jl")
include("dynamics/sixdof_quaternion_fixed_mass.jl")
include("dynamics/sixdof_ecef_quaternion_fixed_mass.jl")

end # module
