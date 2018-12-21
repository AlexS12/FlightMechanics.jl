using FlightMechanics
import Base: +


export RigidSolid, PointMass,
    get_mass, get_cg, get_inertia, +


# TODO: provide outer constructor checking dimensions?
"""
    RigidSolid(mass::Number, cg::Array{T, 1}, inertia::Array{T, 2})

Rigid solid represented by its mass center of gravity position reference system
and inertia tensor.
"""
struct RigidSolid
    mass::Number
    cg::Array{T, 1} where T<:Number
    inertia::Array{T, 2} where T<:Number
end

# Getters
get_mass(rs::RigidSolid) = rs.mass
get_cg(rs::RigidSolid) = rs.cg
get_inertia(rs::RigidSolid) = rs.inertia


"""
    PointMass(mass::Number, position::Array{T, 1})

Point mass represented by its mass and position. It is represented by a
`RigidSolid` with null inertia tensor.
"""
function PointMass(mass::Number, position::Array{T, 1}) where T<:Number
    RigidSolid(mass, position, zeros(3, 3))
end


"""
    get_inertia(rs::RigidSolid, point::Array{T, 1}) where T<:Number

Calculate inertia tensor of a `RigidSolid` object in a point P using Steiner's
theorem.
"""
function get_inertia(rs::RigidSolid, point::Array{T, 1}) where T<:Number
    steiner_inertia(rs.cg, rs.inertia, rs.mass, point)
end


"""
    +(comp1::RigidSolid, comp2::RigidSolid)

Compound two `RigidSolid` objects. Masses are added, a new cg is calculated and
the inertia tensors are added after being expressed with respect to the new
center of gravity using Steiner's theorem.
"""
function +(comp1::RigidSolid, comp2::RigidSolid)
    mass = comp1.mass + comp2.mass
    cg = (comp1.cg*comp1.mass + comp2.cg*comp2.mass) / (comp1.mass + comp2.mass)
    inertia = get_inertia(comp1, cg) + get_inertia(comp2, cg)
    RigidSolid(mass, cg, inertia)
end
