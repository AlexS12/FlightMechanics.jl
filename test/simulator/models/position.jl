using FlightMechanics.Models


@static if VERSION < v"0.7.0-DEV.2005"
    using Base.Test
else
    using Test
end

llh = [deg2rad(49.996908), 0.000000, 9907.31]
xyz_ecef = [4114496.258, 0.0, 4870157.031]
xyz_earth = [0., 0., -9907.31]

pllh = PositionLLH(llh...)

@test isapprox(get_llh(pllh), llh)
@test isapprox(get_xyz_earth(pllh), xyz_earth)
@test isapprox(get_xyz_ecef(pllh), xyz_ecef)
@test isapprox(get_height(pllh), llh[3])

xyz_earth = [0., 0., -9907.31]
pearth = PositionEarth(xyz_earth..., llh[1:2]...)

@test isapprox(get_llh(pearth), llh)
@test isapprox(get_xyz_earth(pearth), xyz_earth)
@test isapprox(get_xyz_ecef(pearth), xyz_ecef)
@test isapprox(get_height(pearth), -xyz_earth[3])


pecef = PositionECEF(xyz_ecef...)

@test isapprox(get_llh(pecef), llh, rtol=1e-5)
@test isapprox(get_xyz_earth(pecef), xyz_earth, rtol=1e-5)
@test isapprox(get_xyz_ecef(pecef), xyz_ecef)
@test isapprox(get_height(pecef), llh[3], atol=0.17)

pzero = Position()

@test isapprox(get_llh(pzero), [0, 0, 0])
@test isapprox(get_xyz_earth(pzero), [0, 0, 0])
@test isapprox(get_xyz_ecef(pzero), llh2ecef(0, 0, 0))
@test isapprox(get_height(pzero), 0)
