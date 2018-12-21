using FlightMechanics.Simulator.Models


@static if VERSION < v"0.7.0-DEV.2005"
    using Base.Test
else
    using Test
end


euler = [1.690, 1.128, 1.867]
quat = [0.655127, 0.212911, 0.719168, 0.0909671]


att = Attitude(euler...)
@test isapprox(get_euler_angles(att), euler)
@test isapprox(get_quaternions(att), quat, atol=1e-5)

att = Attitude(quat...)
@test isapprox(get_euler_angles(att), euler, atol=1e-5)
@test isapprox(get_quaternions(att), quat)
