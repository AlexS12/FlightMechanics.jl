export Aerodynamics, Aircraft


abstract type Aerodynamics end


struct Aircraft
    mass_props::RigidSolid
    pfm::PointForcesMoments

    aerodynamics::Aerodynamics
    propulsion::Propulsion
    fcs::FCS
end
