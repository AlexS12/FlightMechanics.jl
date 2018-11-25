import Base: +, -, *, isapprox
using LinearAlgebra


export PointForcesMoments, translate_forces_moments, +


# TODO: provide outer constructor checking dimensions?
# TODO: think about associating it to a coordinate system
struct PointForcesMoments
    point::Array{Number, 1}
    forces::Array{Number, 1}
    moments::Array{Number, 1}
end

function translate_forces_moments(pfm::PointForcesMoments, point::Array{T, 1}) where T<:Number
    r = pfm.point - point
    if isapprox(r, [0., 0., 0.])
        return pfm
    else
        f = pfm.forces
        m = pfm.moments + cross(r, f)
        return PointForcesMoments(point, f, m)
    end
end

function +(pfm1::PointForcesMoments, pfm2::PointForcesMoments, point::Array{T, 1}) where T<:Number

    pfm1 = translate_forces_moments(pfm1, point)
    pfm2 = translate_forces_moments(pfm2, point)
    PointForcesMoments(point, pfm1.forces+pfm2.forces, pfm1.moments+pfm2.moments)
end

function +(pfm1::PointForcesMoments, pfm2::PointForcesMoments)
    +(pfm1, pfm2, pfm1.point)
end

function *(pfm::PointForcesMoments, n::Number)
    PointForcesMoments(pfm.point, n*pfm.forces, n*pfm.moments)
end

*(n::Number, pfm::PointForcesMoments) = *(pfm, n)

function -(pfm1::PointForcesMoments, pfm2::PointForcesMoments, point::Array{T, 1}) where T<:Number
    +(pfm1, (-1) * pfm2, point)
end

function -(pfm1::PointForcesMoments, pfm2::PointForcesMoments)
    -(pfm1, pfm2, pfm1.point)
end

function isapprox(pfm1::PointForcesMoments, pfm2::PointForcesMoments; kwargs...)
    trans_pfm2 = translate_forces_moments(pfm2, pfm1.point)
    cond = isapprox(pfm1.forces, trans_pfm2.forces; kwargs...) &
           isapprox(pfm1.moments, trans_pfm2.moments; kwargs...)
    return cond
end

function rotate(pfm::PointForcesMoments, psi, theta, phi)
    pfm = PointForcesMoments(pfm.point,
                             body2hor(pfm.forces..., psi, theta, phi),
                             body2hor(pfm.moments..., psi, theta, phi)
                             )
end
