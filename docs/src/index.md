# Flight Mechanics

# FlightMechanics.jl API REFERENCE


```@contents
```

## Functions

### Atmosphere ISA (atmosphere.jl)

```@docs
FlightMechanics.atmosphere_isa(height::Real)
```

### Coordinates Systems (coordinates.jl)

```@docs
FlightMechanics.body2hor(xb, yb, zb, psi, theta, phi)
FlightMechanics.hor2body(xh, yh, zh, psi, theta, phi)
FlightMechanics.wind2hor(xw, yw, zw, chi, gamma, mu)
FlightMechanics.hor2wind(xh, yh, zh, chi, gamma, mu)
FlightMechanics.body2wind(xb, yb, zb, alpha, beta)
FlightMechanics.wind2body(xw, yw, zw, alpha, beta)
```

## Index
```@index
```
