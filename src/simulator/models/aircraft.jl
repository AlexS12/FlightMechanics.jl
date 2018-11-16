export Aerodynamics, Aircraft


abstract type Aerodynamics end

abstract type Aircraft end


get_mass_props(ac::Aircraft) = ac.mass_props
get_pfm(ac::Aircraft) = ac.get_mass_props
get_aerodynamics(ac::Aircraft) = ac.aerodynamics
get_propulsion(ac::Aircraft) = ac.propulsion
