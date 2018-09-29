# API reference

## ISA (atmosphere.jl)
```@autodocs
Modules = [FlightMechanics]
Pages   = ["atmosphere.jl"]
```

## Coordinates Systems (coordinates.jl)

```@autodocs
Modules = [FlightMechanics]
Pages   = ["coordinates.jl"]
```

## Anemometry (anemometry.jl)
```@autodocs
Modules = [FlightMechanics]
Pages   = ["anemometry.jl"]
```

## Kinematics (kinematics.jl)
```@docs
FlightMechanics.rigid_body_velocity(vel_P, ω, r_PQ)
FlightMechanics.rigid_body_acceleration(acc_P, ω, ω_dot, r_PQ)
```

## 6 Degrees of Freedom dynamic models (dynamics/)

```@autodocs
Modules = [FlightMechanics]
Pages   = ["dynamics/sixdof_euler_fixed_mass.jl",
           "dynamics/sixdof_quaternion_fixed_mass.jl",
           "dynamics/sixdof_ecef_quaternion_fixed_mass.jl"
          ]
```
