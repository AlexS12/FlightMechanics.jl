using FlightMechanics.Simulator.Models


@static if VERSION < v"0.7.0-DEV.2005"
    using Base.Test
else
    using Test
end


# Constructor
rs = RigidSolid(15.0, [1.0, 1.0, 1.0], Matrix(1.0I, 3, 3))
@test isapprox(15.0, rs.mass)
@test isapprox([1.0, 1.0, 1.0], rs.cg)
@test isapprox(Matrix(1.0I, 3, 3), rs.inertia)
