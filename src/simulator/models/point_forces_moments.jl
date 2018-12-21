import Base: +, -, *, isapprox
using LinearAlgebra


export PointForcesMoments, translate_forces_moments, +, -, *, isapprox, rotate


# TODO: provide outer constructor checking dimensions?
# TODO: think about associating it to a coordinate system
"""
    PointForcesMoments

Forces and moments applied on a point. This objects can be added, translated or
rotated to be expressed in other coordinate system.

# Constructor

    PointForcesMoments(point::Array{Number, 1}, forces::Array{Number, 1},
                       moments::Array{Number, 1})
"""
struct PointForcesMoments
    point::Array{Number, 1}
    forces::Array{Number, 1}
    moments::Array{Number, 1}
end

"""
    translate_forces_moments(pfm::PointForcesMoments, point::Array{T, 1}) where T<:Number

Calculate equivalent `PointForcesMoments` in point Q given the forces and
moments at point P. Forces are preserved and moments are calculated taking into
account the arm between the two points.

``M_{Q} = M_{P} + r \times F``
"""
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

"""
    +(pfm1::PointForcesMoments, pfm2::PointForcesMoments, point::Array{T, 1})
      where T<:Number

Add two PointForcesMoments objects expressing the result in point P.
"""
function +(pfm1::PointForcesMoments, pfm2::PointForcesMoments, point::Array{T, 1}) where T<:Number

    pfm1 = translate_forces_moments(pfm1, point)
    pfm2 = translate_forces_moments(pfm2, point)
    PointForcesMoments(point, pfm1.forces+pfm2.forces, pfm1.moments+pfm2.moments)
end

"""
Add two PointForcesMoments objects. The result will be expressed in the first
point.
"""
function +(pfm1::PointForcesMoments, pfm2::PointForcesMoments)
    +(pfm1, pfm2, pfm1.point)
end

"""
Multiply the forces and moments by a number.
"""
function *(pfm::PointForcesMoments, n::Number)
    PointForcesMoments(pfm.point, n*pfm.forces, n*pfm.moments)
end

*(n::Number, pfm::PointForcesMoments) = *(pfm, n)

"""
    -(pfm1::PointForcesMoments, pfm2::PointForcesMoments, point::Array{T, 1})
      where T<:Number

Substract two PointForcesMoments objects expressing the result in point P.
"""
function -(pfm1::PointForcesMoments, pfm2::PointForcesMoments, point::Array{T, 1}) where T<:Number
    +(pfm1, (-1) * pfm2, point)
end

"""
Substract two PointForcesMoments objects. The result will be expressed in the first
point.
"""
function -(pfm1::PointForcesMoments, pfm2::PointForcesMoments)
    -(pfm1, pfm2, pfm1.point)
end

"""
Check if two PointForcesMoments are equivalent when expressed at the same point.
"""
function isapprox(pfm1::PointForcesMoments, pfm2::PointForcesMoments; kwargs...)
    trans_pfm2 = translate_forces_moments(pfm2, pfm1.point)
    cond = isapprox(pfm1.forces, trans_pfm2.forces; kwargs...) &
           isapprox(pfm1.moments, trans_pfm2.moments; kwargs...)
    return cond
end

"""
    rotate(pfm::PointForcesMoments, psi, theta, phi)

Express the forces and moments in other coordinate system using the Euler angles
that orientate the original pfm with respect to the second.
"""
function rotate(pfm::PointForcesMoments, psi, theta, phi)
    pfm = PointForcesMoments(pfm.point,
                             body2hor(pfm.forces..., psi, theta, phi),
                             body2hor(pfm.moments..., psi, theta, phi)
                             )
end
