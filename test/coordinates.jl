import FlightMechanics: body2hor, hor2body, wind2hor, hor2wind, wind2body,
                        body2wind

@static if VERSION < v"0.7.0-DEV.2005"
    using Base.Test
else
    using Test
end

ones_ = [1.0, 1.0, 1.0]

# body2hor
@test ones_ ≈ body2hor(ones_..., 0., 0., 0.)
@test [2*0.70710678118654757, 1, 0] ≈ body2hor(ones_..., 0., 45*pi/180., 0.)
@test [1, 0, 2*0.70710678118654757] ≈ body2hor(ones_..., 0., 0., 45*pi/180)
@test [0, 2*0.70710678118654757, 1] ≈ body2hor(ones_..., 45*pi/180, 0., 0.)
# hor2body
@test ones_ ≈ hor2body(ones_..., 0., 0., 0.)
@test ones_ ≈ hor2body(2*0.70710678118654757, 1, 0, 0., 45*pi/180., 0.)
@test ones_ ≈ hor2body(1, 0, 2*0.70710678118654757, 0., 0., 45*pi/180)
@test ones_ ≈ hor2body(0, 2*0.70710678118654757, 1, 45*pi/180, 0., 0.)
#wind2hor
@test ones_ ≈ wind2hor(ones_..., 0., 0., 0.)
@test [2*0.70710678118654757, 1, 0] ≈ wind2hor(ones_..., 0., 45*pi/180., 0.)
@test [1, 0, 2*0.70710678118654757] ≈ wind2hor(ones_..., 0., 0., 45*pi/180)
@test [0, 2*0.70710678118654757, 1] ≈ wind2hor(ones_..., 45*pi/180, 0., 0.)
#hor2wind
@test ones_ ≈ hor2wind(ones_..., 0., 0., 0.)
@test ones_ ≈ hor2wind(2*0.70710678118654757, 1, 0, 0., 45*pi/180., 0.)
@test ones_ ≈ hor2wind(1, 0, 2*0.70710678118654757, 0., 0., 45*pi/180)
@test ones_ ≈ hor2wind(0, 2*0.70710678118654757, 1, 45*pi/180, 0., 0.)
#wind2body
@test ones_ ≈ wind2body(ones_..., 0., 0.)
@test [0, 1, 2*0.70710678118654757] ≈ wind2body(ones_..., 45*pi/180., 0.)
@test [0, 2*0.70710678118654757, 1] ≈ wind2body(ones_..., 0., 45*pi/180.)
#body2wind
@test ones_ ≈ body2wind(ones_..., 0., 0.)
@test ones_ ≈ body2wind(0, 1, 2*0.70710678118654757, 45*pi/180., 0.)
@test ones_ ≈ body2wind(0, 2*0.70710678118654757, 1, 0., 45*pi/180.)

# llh ECEF (using data from
# .. [1] Bowring, B. R. (1976). Transformation from spatial to geographical 
# coordinates. Survey review, 23(181), 323-327.)
import FlightMechanics: ecef2llh, llh2ecef
import FlightMechanics.EarthConstants: Ellipsoid

# From [1] Example 1 (page 325)
x = 4114496.258  # m
y = 0.0          # m
z = 4870157.031  # m
# Ellipsoid does not match any known standard
a_bowring1976 = 6378249.145  # m
b_bowring1976 = 6356514.870  # m
finv_bowring1976 = a_bowring1976 / (a_bowring1976 - b_bowring1976)
Bowring1976 = Ellipsoid(a_bowring1976, finv_bowring1976)

exp_llh = [deg2rad(50.0), 0, 10000.0]
llh = ecef2llh(x, y, z; ellipsoid=Bowring1976)
@test isapprox(llh, exp_llh)

exp_xyz = [4114496.258, 0.0, 4870157.031]  # m
xyz = llh2ecef(deg2rad(50.0), 0, 10000.0; ellipsoid=Bowring1976)
@test isapprox(xyz, exp_xyz)


# 
x, y, z = (4114496.258, 0.0, 4870157.031)  # m
exp_llh = [deg2rad(49.996908), 0.000000, 9907.31]
llh = ecef2llh(x, y, z)
@test isapprox(llh[:2], exp_llh[:2], atol=0.0018/3600)

exp_xyz = [4114291.97, 0.00, 4870449.48]  # m
xyz = llh2ecef(deg2rad(50.0), 0, 10000.0)
@test isapprox(xyz, exp_xyz)