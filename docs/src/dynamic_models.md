# Dynamic System Models

Every dynamics system should have a common interface composed of:
    - Mandatory: `state, mass, inertia, forces, moments`
    - Optional: additional angular momentum contributions, `h`, (zero otherwise)
    - Method specific: k, ellipsoid...

## Six Degrees of Freedom (6DOF)

### Euler Flat Earth Fixed mass

Six degrees of freedom dynamic system using Euler angles for attitude
representation and assuming fixed mass.

Flat Earth hypothesis is applied and Earth reference frame is considered
inertial.

It is considered that the aircraft xb-zb plane is a plane of symmetry so that
Jxy and Jyz cross-product of inertia are zero and will not be taken into
account.

The effects of the angular momentum produced by spinning rotors is taken into
account with the optional argument `h`.

TODO: complete with equations and assumptions

### Quaternion Flat Earth Fixed mass

Six degrees of freedom dynamic system using quaternions for attitude
representation and assuming fixed mass.

Flat Earth hypothesis is applied and Earth reference frame is considered
inertial.

It is considered that the aircraft xb-zb plane is a plane of symmetry so that
Jxy and Jyz cross-product of inertia are zero and will not be taken into
account.

The effects of the angular momentum produced by spinning rotors is taken into
account with the optional argument `h`.

TODO: complete with equations and assumptions

### Quaternion Ellipsoidal Earth Fixed mass

Six degrees of freedom dynamic system using quaternions for attitude
representation and assuming fixed mass.

Ellipsoidal Earth Model is used and the ECEF reference frame is considered
inertial.

The effects of the angular momentum produced by spinning rotors is taken into
account with the optional argument `h`.

TODO: complete with equations and assumptions
