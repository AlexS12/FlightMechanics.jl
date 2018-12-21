using LinearAlgebra

using FlightMechanics


@static if VERSION < v"0.7.0-DEV.2005"
    using Base.Test
else
    using Test
end


@testset "quaternion <-> euler" begin
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
    quat3 = euler2quaternion(euler...)
    @test isapprox(quat, quat3)
end


@testset "body <-> horizon" begin
    ones_ = [1.0, 1.0, 1.0]

    # body2hor Euler
    # no rotation
    @test ones_ ≈ body2hor(ones_..., 0., 0., 0.)

    angles1 = [0., 45*pi/180., 0.]
    angles2 = [0., 0., 45*pi/180]
    angles3 = [45*pi/180, 0., 0.]
    exp_b2h_1 = [2*0.70710678118654757, 1, 0]
    exp_b2h_2 = [1, 0, 2*0.70710678118654757]
    exp_b2h_3 = [0, 2*0.70710678118654757, 1]

    @test exp_b2h_1 ≈ body2hor(ones_..., angles1...)
    @test exp_b2h_2 ≈ body2hor(ones_..., angles2...)
    @test exp_b2h_3 ≈ body2hor(ones_..., angles3...)

    # hor2body Euler
    @test ones_ ≈ hor2body(ones_..., 0., 0., 0.)
    @test ones_ ≈ hor2body(exp_b2h_1..., angles1...)
    @test ones_ ≈ hor2body(exp_b2h_2..., angles2...)
    @test ones_ ≈ hor2body(exp_b2h_3..., angles3...)

    # body2hor quaternion
    @test ones_ ≈ body2hor(ones_..., 0., 0., 0.)

    quat1 = euler2quaternion(angles1...)
    quat2 = euler2quaternion(angles2...)
    quat3 = euler2quaternion(angles3...)

    @test exp_b2h_1 ≈ body2hor(ones_..., quat1...)
    @test exp_b2h_2 ≈ body2hor(ones_..., quat2...)
    @test exp_b2h_3 ≈ body2hor(ones_..., quat3...)

    # hor2body quaternionr
    @test ones_ ≈ hor2body(ones_..., 0., 0., 0.)
    @test ones_ ≈ hor2body(exp_b2h_1...,  quat1...)
    @test ones_ ≈ hor2body(exp_b2h_2...,  quat2...)
    @test ones_ ≈ hor2body(exp_b2h_3...,  quat3...)

    # rotation matrix body to hor (euler)
    r_hb1 = rot_matrix_body2hor(angles1...)
    @test exp_b2h_1 ≈ r_hb1 * ones_
    r_hb2 = rot_matrix_body2hor(angles2...)
    @test exp_b2h_2 ≈ r_hb2 * ones_
    r_hb3 = rot_matrix_body2hor(angles3...)
    @test exp_b2h_3 ≈ r_hb3 * ones_

    # rotation matrix body <-> hor (quaternion)
    r_hb1q = rot_matrix_body2hor(quat1...)
    @test exp_b2h_1 ≈ r_hb1q * ones_
    r_hb2q = rot_matrix_body2hor(quat2...)
    @test exp_b2h_2 ≈ r_hb2q * ones_
    r_hb3q = rot_matrix_body2hor(quat3...)
    @test exp_b2h_3 ≈ r_hb3q * ones_

    # r_hb equivalent with euler and quaternion
    @test isapprox(r_hb1, r_hb1q)
    @test isapprox(r_hb2, r_hb2q)
    @test isapprox(r_hb3, r_hb3q)

    # rotation matrix hor to body (euler)
    r_bh1 = rot_matrix_hor2body(angles1...)
    @test ones_ ≈ r_bh1 * exp_b2h_1
    r_bh2 = rot_matrix_hor2body(angles2...)
    @test ones_ ≈ r_bh2 * exp_b2h_2
    r_bh3 = rot_matrix_hor2body(angles3...)
    @test ones_ ≈ r_bh3 * exp_b2h_3

    # rotation matrix hor2body (quaternion)
    r_bh1q = rot_matrix_hor2body(quat1...)
    @test ones_ ≈ r_bh1q * exp_b2h_1
    r_bh2q = rot_matrix_hor2body(quat2...)
    @test ones_ ≈ r_bh2q * exp_b2h_2
    r_bh3q = rot_matrix_hor2body(quat3...)
    @test ones_ ≈ r_bh3q * exp_b2h_3

    # r_bh equivalent with euler and quaternion
    @test isapprox(r_bh1, r_bh1q)
    @test isapprox(r_bh2, r_bh2q)
    @test isapprox(r_bh3, r_bh3q)

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
end


@testset "wind <-> hor/body" begin
    ones_ = [1.0, 1.0, 1.0]
    angles1 = [0., 45*pi/180., 0.]
    angles2 = [0., 0., 45*pi/180]
    angles3 = [45*pi/180, 0., 0.]
    exp_b2h_1 = [2*0.70710678118654757, 1, 0]
    exp_b2h_2 = [1, 0, 2*0.70710678118654757]
    exp_b2h_3 = [0, 2*0.70710678118654757, 1]
    #wind2hor
    @test ones_ ≈ wind2hor(ones_..., 0., 0., 0.)
    @test exp_b2h_1 ≈ wind2hor(ones_..., angles1...)
    @test exp_b2h_2 ≈ wind2hor(ones_..., angles2...)
    @test exp_b2h_3 ≈ wind2hor(ones_..., angles3...)
    #hor2wind
    @test ones_ ≈ hor2wind(ones_..., 0., 0., 0.)
    @test ones_ ≈ hor2wind(exp_b2h_1..., angles1...)
    @test ones_ ≈ hor2wind(exp_b2h_2..., angles2...)
    @test ones_ ≈ hor2wind(exp_b2h_3..., angles3...)
    #wind2body
    @test ones_ ≈ wind2body(ones_..., 0., 0.)
    @test [0, 1, 2*0.70710678118654757] ≈ wind2body(ones_..., 45*pi/180., 0.)
    @test exp_b2h_3 ≈ wind2body(ones_..., 0., 45*pi/180.)
    #body2wind
    @test ones_ ≈ body2wind(ones_..., 0., 0.)
    @test ones_ ≈ body2wind(0, 1, 2*0.70710678118654757, 45*pi/180., 0.)
    @test ones_ ≈ body2wind(exp_b2h_3..., 0., 45*pi/180.)
end


@testset "hor <-> ECEF" begin
    xecef, yecef, zecef = 1.0, 10.0, 100.0
    lat, lon = 0.0, 0.0
    exp_xyz_hor = [100.0, 10.0 ,-1.0]

    xyz_hor =  ecef2hor(xecef, yecef, zecef, lat, lon)
    @test isapprox(xyz_hor, exp_xyz_hor)

    r_hecef = rot_matrix_ecef2hor(lat, lon)
    @test isapprox(exp_xyz_hor, r_hecef * [xecef, yecef, zecef])

    exp_xyz_ecef = [xecef, yecef, zecef]
    xyz_ecef = hor2ecef(exp_xyz_hor..., lat, lon)
    @test isapprox(xyz_ecef, exp_xyz_ecef)

    r_ecefh = rot_matrix_hor2ecef(lat, lon)
    @test isapprox(exp_xyz_ecef, r_ecefh * exp_xyz_hor)

    lat, lon = pi/2.0, 0.0
    exp_xyz_hor = [-1.0, 10.0 ,-100.0]

    xyz_hor =  ecef2hor(xecef, yecef, zecef, lat, lon)
    @test isapprox(xyz_hor, exp_xyz_hor)

    r_hecef = rot_matrix_ecef2hor(lat, lon)
    @test isapprox(exp_xyz_hor, r_hecef * [xecef, yecef, zecef])

    exp_xyz_ecef = [xecef, yecef, zecef]
    xyz_ecef = hor2ecef(exp_xyz_hor..., lat, lon)
    @test isapprox(xyz_ecef, exp_xyz_ecef)

    r_ecefh = rot_matrix_hor2ecef(lat, lon)
    @test isapprox(exp_xyz_ecef, r_ecefh * exp_xyz_hor)

    lat, lon = 0.0, pi/2.0
    exp_xyz_hor = [100.0, -1.0 ,-10.0]

    xyz_hor =  ecef2hor(xecef, yecef, zecef, lat, lon)
    @test isapprox(xyz_hor, exp_xyz_hor)

    r_hecef = rot_matrix_ecef2hor(lat, lon)
    @test isapprox(exp_xyz_hor, r_hecef * [xecef, yecef, zecef])

    exp_xyz_ecef = [xecef, yecef, zecef]
    xyz_ecef = hor2ecef(exp_xyz_hor..., lat, lon)
    @test isapprox(xyz_ecef, exp_xyz_ecef)

    r_ecefh = rot_matrix_hor2ecef(lat, lon)
    @test isapprox(exp_xyz_ecef, r_ecefh * exp_xyz_hor)

    lat, lon = pi/2.0, pi/2.0
    exp_xyz_hor = [-10.0, -1.0 ,-100.0]

    xyz_hor =  ecef2hor(xecef, yecef, zecef, lat, lon)
    @test isapprox(xyz_hor, exp_xyz_hor)

    r_hecef = rot_matrix_ecef2hor(lat, lon)
    @test isapprox(exp_xyz_hor, r_hecef * [xecef, yecef, zecef])

    exp_xyz_ecef = [xecef, yecef, zecef]
    xyz_ecef = hor2ecef(exp_xyz_hor..., lat, lon)
    @test isapprox(xyz_ecef, exp_xyz_ecef)

    r_ecefh = rot_matrix_hor2ecef(lat, lon)
    @test isapprox(exp_xyz_ecef, r_ecefh * exp_xyz_hor)
end

@testset "body <-> ECEF" begin
    # uses ecef2hor and hor2body so not intensive testing necessary
    xecef, yecef, zecef = 1.0, 10.0, 100.0
    psi, theta, phi = pi/4.0, pi/6.0, pi/12.0
    quat = euler2quaternion(psi, theta, phi)
    lat, lon = pi/3, pi/6
    xh, yh, zh = ecef2hor(xecef, yecef, zecef, lat, lon)
    exp_xyz_b = hor2body(xh, yh, zh, psi, theta, phi)

    xyz_body = ecef2body(xecef, yecef, zecef, lat, lon , psi, theta, phi)
    @test isapprox(xyz_body, exp_xyz_b)

    r_becef = rot_matrix_ecef2body(lat, lon , psi, theta, phi)
    @test isapprox(r_becef * [xecef, yecef, zecef], exp_xyz_b)

    xyz_body = ecef2body(xecef, yecef, zecef, lat, lon , quat...)
    @test isapprox(xyz_body, exp_xyz_b)

    r_becef_q = rot_matrix_ecef2body(lat, lon , quat...)
    @test isapprox(r_becef_q * [xecef, yecef, zecef], exp_xyz_b)

    exp_xyz_ecef = [xecef, yecef, zecef]

    xyz_ecef = body2ecef(exp_xyz_b..., lat, lon, psi, theta, phi)
    @test isapprox(xyz_ecef, exp_xyz_ecef)

    r_ecefb = rot_matrix_body2ecef(lat, lon , psi, theta, phi)
    @test isapprox(r_ecefb * exp_xyz_b, exp_xyz_ecef)

    xyz_ecef = body2ecef(exp_xyz_b..., lat, lon, quat...)
    @test isapprox(xyz_ecef, exp_xyz_ecef)

    r_ecefb_q = rot_matrix_body2ecef(lat, lon , quat...)
    @test isapprox(r_ecefb_q * exp_xyz_b, exp_xyz_ecef)
end


@testset "llh <-> ECEF" begin
    # llh ECEF (using data from
    # - [1] Bowring, B. R. (1976). Transformation from spatial to geographical
    # coordinates. Survey review, 23(181), 323-327.)
    using FlightMechanics: Ellipsoid

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
end
