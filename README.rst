FlightMechanics
===============
.. |travisci| image:: https://travis-ci.org/AlexS12/FlightMechanics.jl.svg?branch=master
    :target: https://travis-ci.org/AlexS12/FlightMechanics.jl

.. |license| image:: https://img.shields.io/badge/license-MIT-blue.svg?style=flat-square
   :target: https://github.com/AlexS12/FlightMechanics.jl/blob/master/LICENSE.md
   
.. |codecov| image:: https://codecov.io/gh/AlexS12/FlightMechanics.jl/branch/master/graph/badge.svg
  :target: https://codecov.io/gh/AlexS12/FlightMechanics.jl
  
.. |docs| image:: https://img.shields.io/badge/docs-latest-brightgreen.svg?style=flat-square
   :target: https://alexs12.github.io/FlightMechanics.jl/latest/
   
.. |logo| image:: https://github.com/AlexS12/FlightMechanics.jl/blob/master/docs/src/logo.png
   :target: https://github.com/AlexS12/FlightMechanics.jl/blob/master/docs/src/logo.png
   
|license| |docs| |travisci| |codecov| 

|logo|

Flight Mechanics utils :airplane:
---------------------------------

\:warning: This package is in its initial development phase :construction:

This package is intended to provide utils for Flight Mechanics computations. It runs on julia ≥ 0.7.0. 

* International Standard Atmosphere: get pressure, temperature, density, sound velocity for a given altitude.

* Conversion between different coordinate systems:

  * body
  * horizon
  * wind
  * ECEF
  * llh (latitude, longitude, height) over various ellipsoid models
  
* Quaternion and euler angles conversions
  
* Anemometry:

  * conversion between tas, cas, eas.
  * velocity calculation from airspeed indicator (ASI) pressure difference
  * dynamic pressure calculation (compressible and incompressible cases)
  * tas, alpha, beta from body velocity
  
* 6 DOF Dynamic fixed mass models:

  * Flat Earth Euler angles
  * Flat Earth quaternions
  * Ellipsoidal Earth ECEF model quaternion

Install :rocket:
----------------

You can install this package cloning it::

  Pkg> add git@github.com:AlexS12/FlightMechanics.jl.git
  
or if you want the dev version::

  Pkg> dev git@github.com:AlexS12/FlightMechanics.jl.git

and run the tests::

  Pkg> test FlightMechanics


Contributing :helicopter:
--------------------------

If you used this package and have any suggestion or found a bug, please `open an issue`_

.. _open an issue: https://github.com/AlexS12/FlightMechanics.jl/issues

If this package is useful for you and want to join efforts don't hesitate to `let me know`_.

.. _let me know: https://github.com/AlexS12

|Ask Me Anything !| |Open Source Love svg2|

.. |Open Source Love svg2| image:: https://badges.frapsoft.com/os/v2/open-source.svg?v=103
   :target: https://github.com/ellerbrock/open-source-badges/
   
.. |Ask Me Anything !| image:: https://img.shields.io/badge/Ask%20me-anything-1abc9c.svg
   :target: https://github.com/AlexS12
