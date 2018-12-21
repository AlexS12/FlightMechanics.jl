using FlightMechanics.Simulator.Models

@static if VERSION < v"0.7.0-DEV.2005"
    using Base.Test
else
    using Test
end


# translate_forces_moments
pfm1 = PointForcesMoments([0, 0 ,0], [1, 0, 0], [0, 0, 0])
trans_pfm1 = translate_forces_moments(pfm1, [0, 0, 1])
exp_pfm1 = PointForcesMoments([0, 0, 1], [1, 0, 0], [0, -1, 0])

@test isapprox(trans_pfm1.point, exp_pfm1.point)
@test isapprox(trans_pfm1.forces, exp_pfm1.forces)
@test isapprox(trans_pfm1.moments, exp_pfm1.moments)

pfm2 = PointForcesMoments([0, 0 ,0], [1, 0, 0], [1, 0, 0])
trans_pfm2 = translate_forces_moments(pfm2, [0, 0, 1])
exp_pfm2 = PointForcesMoments([0, 0, 1], [1, 0, 0], [1, -1, 0])

@test isapprox(trans_pfm2.point, exp_pfm2.point)
@test isapprox(trans_pfm2.forces, exp_pfm2.forces)
@test isapprox(trans_pfm2.moments, exp_pfm2.moments)

# isapprox
@test isapprox(pfm1, trans_pfm1)
@test ≈(pfm1, trans_pfm1)
@test isapprox(pfm2, trans_pfm2)
@test ≈(pfm2, trans_pfm2)

# +
exp_pfm_sum = PointForcesMoments([0, 0, 0], [2, 0, 0], [0, 0, 0])
@test isapprox(pfm1 + pfm1, exp_pfm_sum)
# -
exp_pfm_sub = PointForcesMoments([0, 0, 0], [0, 0, 0], [0, 0, 0])
@test isapprox(pfm2 - pfm2, exp_pfm_sub)
# *
exp_pfm_mul = PointForcesMoments([0, 0, 1], [2, 0, 0], [2, -2, 0])
@test isapprox(2 * exp_pfm2, exp_pfm_mul)

# rotate
pfm3 = PointForcesMoments([0, 0, 0], [1, -1, 1], [-1, 1, 1])
rot_pfm3 = rotate(pfm3, 0, 0, 0)
exp_rot_pfm3 = PointForcesMoments([0, 0, 0], [1, -1, 1], [-1, 1, 1])
@test isapprox(rot_pfm3, exp_rot_pfm3)

pfm3 = PointForcesMoments([0, 0, 0], [0, 1, 0], [-1, 0, 0])
rot_pfm3 = rotate(pfm3, -π/2, 0, 0)
exp_rot_pfm3 = PointForcesMoments([0, 0, 0], [1, 0, 0], [0, 1, 0])
@test isapprox(rot_pfm3, exp_rot_pfm3, atol=1e-15)

pfm3 = PointForcesMoments([0, 0, 0], [0, 1, 0], [-1, 0, 0])
rot_pfm3 = rotate(pfm3, 0, π/2, 0)
exp_rot_pfm3 = PointForcesMoments([0, 0, 0], [0, 1, 0], [0, 0, 1])
@test isapprox(rot_pfm3, exp_rot_pfm3, atol=1e-15)
