using LinearAlgebra
import FlightMechanics: body2hor, hor2body, wind2hor, hor2wind, wind2body,
                        body2wind

@static if VERSION < v"0.7.0-DEV.2005"
    using Base.Test
else
    using Test
end

# Quaternion <-> Euler angles
import FlightMechanics: quaternion2euler, euler2quaternion

quat = (0.8660254037844387, 0.0, 0.5, 0.0)
euler_exp = [0.0, 1.04719755, 0.0]
euler = quaternion2euler(quat...)
@test isapprox(euler, euler_exp)
quat_exp = [quat...]
quat = euler2quaternion(euler_exp...)
@test isapprox(quat, quat_exp)

quat = [0.5, 0.5, 0.0, 0.0]
euler_exp = [0.0, 0.0, pi/2.0]
euler = quaternion2euler(quat...)
@test isapprox(euler, euler_exp)
quat_exp = [quat...]
quat = euler2quaternion(euler_exp...)
@test isapprox(quat, quat_exp / norm(quat_exp))

psi, theta, phi = pi/4.0, pi/6.0, pi/12.0
quat = euler2quaternion(psi, theta, phi)
euler = quaternion2euler(quat...)
@test isapprox([psi, theta, phi], euler)

quat = [0.5, 0.1, 0.2, 0.7]
quat = quat / norm(quat)
euler = quaternion2euler(quat...)
quat2 = euler2quaternion(euler...)
@test isapprox(quat, quat2)

ones_ = [1.0, 1.0, 1.0]

# body2hor Euler
@test ones_ ≈ body2hor(ones_..., 0., 0., 0.)
@test [2*0.70710678118654757, 1, 0] ≈ body2hor(ones_..., 0., 45*pi/180., 0.)
@test [1, 0, 2*0.70710678118654757] ≈ body2hor(ones_..., 0., 0., 45*pi/180)
@test [0, 2*0.70710678118654757, 1] ≈ body2hor(ones_..., 45*pi/180, 0., 0.)
# hor2body Euler
@test ones_ ≈ hor2body(ones_..., 0., 0., 0.)
@test ones_ ≈ hor2body(2*0.70710678118654757, 1, 0, 0., 45*pi/180., 0.)
@test ones_ ≈ hor2body(1, 0, 2*0.70710678118654757, 0., 0., 45*pi/180)
@test ones_ ≈ hor2body(0, 2*0.70710678118654757, 1, 45*pi/180, 0., 0.)
# body2hor quaternion
@test ones_ ≈ body2hor(ones_..., 0., 0., 0.)
quat0 = euler2quaternion(0., 45*pi/180., 0.)
quat1 = euler2quaternion(0., 0., 45*pi/180.)
quat2 = euler2quaternion(45*pi/180., 0., 0.)
@test [2*0.70710678118654757, 1, 0] ≈ body2hor(ones_..., quat0...)
@test [1, 0, 2*0.70710678118654757] ≈ body2hor(ones_..., quat1...)
@test [0, 2*0.70710678118654757, 1] ≈ body2hor(ones_..., quat2...)
# hor2body quaternionr
@test ones_ ≈ hor2body(ones_..., 0., 0., 0.)
@test ones_ ≈ hor2body(2*0.70710678118654757, 1, 0,  quat0...)
@test ones_ ≈ hor2body(1, 0, 2*0.70710678118654757,  quat1...)
@test ones_ ≈ hor2body(0, 2*0.70710678118654757, 1,  quat2...)

# Test that quaternion and euler produce the same transformation
psi, theta, phi = pi/4.0, pi/6.0, pi/12.0
quat = euler2quaternion(psi, theta, phi)
xh, yh, zh = 100.0, 10.0, -1.0
xyz_b_e = hor2body(xh, yh, zh, psi, theta, phi)
xyz_b_q = hor2body(xh, yh, zh, quat...)
@test isapprox(xyz_b_e, xyz_b_q)

xyz_h_e = body2hor(xyz_b_e..., psi, theta, phi)
xyz_h_q = body2hor(xyz_b_e..., quat...)
@test isapprox(xyz_h_e, xyz_h_q)


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

# Hor <-> ECEF
import FlightMechanics: ecef2hor, hor2ecef
xecef, yecef, zecef = 1.0, 10.0, 100.0
lat, lon = 0.0, 0.0
exp_xyz_hor = [100.0, 10.0 ,-1.0]
xyz_hor =  ecef2hor(xecef, yecef, zecef, lat, lon)
@test isapprox(xyz_hor, exp_xyz_hor)
exp_xyz_ecef = [xecef, yecef, zecef]
xyz_ecef = hor2ecef(exp_xyz_hor..., lat, lon)
@test isapprox(xyz_ecef, exp_xyz_ecef)

lat, lon = pi/2.0, 0.0
exp_xyz_hor = [-1.0, 10.0 ,-100.0]
xyz_hor =  ecef2hor(xecef, yecef, zecef, lat, lon)
@test isapprox(xyz_hor, exp_xyz_hor)
exp_xyz_ecef = [xecef, yecef, zecef]
xyz_ecef = hor2ecef(exp_xyz_hor..., lat, lon)
@test isapprox(xyz_ecef, exp_xyz_ecef)

lat, lon = 0.0, pi/2.0
exp_xyz_hor = [100.0, -1.0 ,-10.0]
xyz_hor =  ecef2hor(xecef, yecef, zecef, lat, lon)
@test isapprox(xyz_hor, exp_xyz_hor)
exp_xyz_ecef = [xecef, yecef, zecef]
xyz_ecef = hor2ecef(exp_xyz_hor..., lat, lon)
@test isapprox(xyz_ecef, exp_xyz_ecef)

lat, lon = pi/2.0, pi/2.0
exp_xyz_hor = [-10.0, -1.0 ,-100.0]
xyz_hor =  ecef2hor(xecef, yecef, zecef, lat, lon)
@test isapprox(xyz_hor, exp_xyz_hor)
exp_xyz_ecef = [xecef, yecef, zecef]
xyz_ecef = hor2ecef(exp_xyz_hor..., lat, lon)
@test isapprox(xyz_ecef, exp_xyz_ecef)

# Body <--> ECEF (uses ecef2hor and hor2body so not intensive testing necessary)
import FlightMechanics: ecef2body, body2ecef
xecef, yecef, zecef = 1.0, 10.0, 100.0
psi, theta, phi = pi/4.0, pi/6.0, pi/12.0
quat = euler2quaternion(psi, theta, phi)
lat, lon = pi/3, pi/6
xh, yh, zh = ecef2hor(xecef, yecef, zecef, lat, lon)
exp_xyz_b = hor2body(xh, yh, zh, psi, theta, phi)

xyz_body = ecef2body(xecef, yecef, zecef, lat, lon , psi, theta, phi)
@test isapprox(xyz_body, exp_xyz_b)
xyz_body = ecef2body(xecef, yecef, zecef, lat, lon , quat...)
@test isapprox(xyz_body, exp_xyz_b)

exp_xyz_ecef = [xecef, yecef, zecef]
xyz_ecef = body2ecef(exp_xyz_b..., lat, lon, psi, theta, phi)
@test isapprox(xyz_ecef, exp_xyz_ecef)
xyz_ecef = body2ecef(exp_xyz_b..., lat, lon, quat...)
@test isapprox(xyz_ecef, exp_xyz_ecef)

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


# http://www.sysense.com/products/ecef_lla_converter/index.html
x, y, z = (4114496.258, 0.0, 4870157.031)  # m
exp_llh = [deg2rad(49.996908), 0.000000, 9907.31]
llh = ecef2llh(x, y, z)
# Longitude should be exact, latitude max error: 0.0018" according to [1]
# Altitude 0.17 m
# The implemented function must be better, but I don't know which one is using
# this web page. The ellipsoid is assumed to be WGS84
@test isapprox(llh[:2], exp_llh[:2], atol=deg2rad(0.0018/3600.0))
@test isapprox(llh[3], exp_llh[3], atol=0.17)

exp_xyz = [4114291.97, 0.00, 4870449.48]  # m
xyz = llh2ecef(deg2rad(50.0), 0, 10000.0)
@test isapprox(xyz, exp_xyz)

# non-zero longitude
x, y, z = (3814496.258, 1514496.258, 4870157.031)  # m
exp_llh = [deg2rad(50.068095), deg2rad(21.654910), 3263.97]
llh = ecef2llh(x, y, z)
# Longitude should be exact, latitude max error: 0.0018" according to [1]
# Altitude 0.17 m
# The implemented function must be better, but I don't know which one is using
# this web page. The ellipsoid is assumed to be WGS84
@test isapprox(llh[:2], exp_llh[:2], atol=deg2rad(0.0018/3600.0))
@test isapprox(llh[3], exp_llh[3], atol=0.17)

exp_xyz = [3814496.258, 1514496.258, 4870157.031]  # m
xyz = llh2ecef(deg2rad(50.068095), deg2rad(21.654910), 3263.97)
@test isapprox(xyz, exp_xyz)