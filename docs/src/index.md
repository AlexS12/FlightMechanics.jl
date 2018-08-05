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

### Anemometry

```@docs
FlightMechanics.qc2cas(qc)
FlightMechanics.qc2tas(qc, ρ, p)
FlightMechanics.qc2eas(qc, p)
FlightMechanics.tas2eas(tas, ρ)
FlightMechanics.eas2tas(eas, ρ)
FlightMechanics.cas2eas(cas, ρ, p)
FlightMechanics.eas2cas(eas, ρ, p)
FlightMechanics.cas2tas(cas, ρ, p)
FlightMechanics.tas2cas(tas, ρ, p)
FlightMechanics.tas_alpha_beta_from_uvw(u, v, w)
FlightMechanics.incompressible_qinf(tas, ρ)
FlightMechanics.compressible_qinf(M, p)
```

## Index
```@index
```
