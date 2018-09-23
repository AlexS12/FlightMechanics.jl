# Home

## Overview

This package is intended to provide utils for Flight Mechanics computations in Julia. Its purpose is to provide reliable, documented, efficient and tested Julia implementations for common calculations in Flight Mechanics such as:

* International Standard Atmosphere: get pressure, temperature, density, sound velocity for a given altitude.
* Conversion between different coordinate systems: body/ABC, horizon/NED, wind, ECEF (Earth Cenetered Earth Fixed), LLH (Latitude, Longitude, Height for various ellipsoids models).
* Support for various attitude representations: Euler angles, quaternions.
* Anemometry calculations: conversions between TAS, CAS, EAS; velocity calculation from airspeed indicator (ASI) pressure difference; dynamic pressure calculation, TAS, $\alpha$, $\beta$ from body velocity.
* 6 DOF Dynamic fixed mass models:
  * Flat Earth Euler angles
  * Flat Earth quaternions
  * Ellipsoidal Earth ECEF model quaternion


For the sake of reliability and reproducibility, several references are listed in the documentation of the different methods so they can be consulted and studied.

## Installing FlightMechanics.jl

This package is still in its initial development phase and has not been registered in the Julia official repository.

You can install this package cloning it:
    
    Pkg> add git@github.com:AlexS12/FlightMechanics.jl.git

You can read more about adding unregistered pakcages in [Julia documentation](https://docs.julialang.org/en/latest/stdlib/Pkg/#Adding-unregistered-packages-1.)
  
or if you are planning to make a contribution and want the dev version:

    Pkg> dev git@github.com:AlexS12/FlightMechanics.jl.git

Once you have installed the package, it is highly recommended to run the tests:

    Pkg> test FlightMechanics

## Contributing

If you used this package and have any suggestion or found a bug, please [open an issue](https://github.com/AlexS12/FlightMechanics.jl/issues).

If this package is useful for you and want to join efforts don't hesitate to let me know: [https://github.com/AlexS12](https://github.com/AlexS12)
