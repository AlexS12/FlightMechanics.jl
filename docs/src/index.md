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
FlightMechanics.body2hor(xb, yb, zb, q0, q1, q2, q3)
FlightMechanics.hor2body(xh, yh, zh, psi, theta, phi)
FlightMechanics.hor2body(xh, yh, zh, q0, q1, q2, q3)
FlightMechanics.rot_matrix_body2hor(psi, theta, phi)
FlightMechanics.rot_matrix_body2hor(q0, q1, q2, q3)
FlightMechanics.rot_matrix_hor2body(psi, theta, phi)
FlightMechanics.rot_matrix_hor2body(q0, q1, q2, q3)
FlightMechanics.wind2hor(xw, yw, zw, chi, gamma, mu)
FlightMechanics.hor2wind(xh, yh, zh, chi, gamma, mu)
FlightMechanics.body2wind(xb, yb, zb, alpha, beta)
FlightMechanics.wind2body(xw, yw, zw, alpha, beta)
FlightMechanics.euler2quaternion(psi, theta, phi)
FlightMechanics.quaternion2euler(q0, q1, q2, q3)
FlightMechanics.ecef2hor(xecef, yecef, zecef, lat, lon)
FlightMechanics.hor2ecef(xh, yh, zh, lat, lon)
FlightMechanics.rot_matrix_ecef2hor(lat, lon)
FlightMechanics.rot_matrix_hor2ecef(lat, lon)
FlightMechanics.ecef2body(xecef, yecef, zecef, lat, lon, psi, theta, phi)
FlightMechanics.ecef2body(xecef, yecef, zecef, lat, lon, q0, q1, q2, q3)
FlightMechanics.body2ecef(xb, yb, zb, lat, lon, psi, theta, phi)
FlightMechanics.body2ecef(xb, yb, zb, lat, lon, q0, q1, q2, q3)
FlightMechanics.rot_matrix_body2ecef(lat, lon, psi, theta, phi)
FlightMechanics.rot_matrix_body2ecef(lat, lon, q0, q1, q2, q3)
FlightMechanics.rot_matrix_ecef2body(lat, lon, psi, theta, phi)
FlightMechanics.rot_matrix_ecef2body(lat, lon, q0, q1, q2, q3)
FlightMechanics.llh2ecef(lat, lon, height; ellipsoid=WGS84)
FlightMechanics.ecef2llh(xecef, yecef, zecef; ellipsoid=WGS84)

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

### 6 Degrees of Freedom dynamic models

```@docs
FlightMechanics.six_dof_euler_fixed_mass(state, mass, inertia, forces, moments)
FlightMechanics.six_dof_quaternion_fixed_mass(state, mass, inertia, forces, moments; k=0.0)
FlightMechanics.six_dof_ecef_quaternion_fixed_mass(state, mass, inertia, forces, moments; k=0.0, ellipsoid=WGS84)
```

## Index
```@index
```
