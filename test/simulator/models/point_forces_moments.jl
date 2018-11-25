using FlightMechanics.Simulator.Models

@static if VERSION < v"0.7.0-DEV.2005"
    using Base.Test
else
    using Test
end


# translate_forces_moments
pfm1 = PointForcesMoments([0, 0 ,0], [1, 0, 0], [0, 0, 0])
trans_pfm1 = translate_forces_moments(pfm1, [0, 0, 1])
exp_pfm = PointForcesMoments([0, 0, 1], [1, 0, 0], [0, -1, 0])

@test isapprox(trans_pfm1.point, exp_pfm.point)
@test isapprox(trans_pfm1.forces, exp_pfm.forces)
@test isapprox(trans_pfm1.moments, exp_pfm.moments)

# isapprox
@test isapprox(pfm1, trans_pfm1)
@test ≈(pfm1, trans_pfm1)
