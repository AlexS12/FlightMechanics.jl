using LinearAlgebra
using FlightMechanics

export EarthConstantGravity, calculate_gravity,
    get_gravity_horizon, get_gravity_accel, get_gravity_body


abstract type Gravity end

get_gravity_horizon(grav::Gravity) = grav.hor
get_gravity_accel(grav::Gravity) = norm(get_gravity_horizon(grav))
get_gravity_body(grav::Gravity, att::Attitude) =
    hor2body(get_gravity_horizon(grav)..., get_quaternions(att)...)


"""
    EarthConstantGravity(value, body_vector)

Earth Constant gravity model with value 9.80665 m/s² and oriented towards
local horizon Z axis.

"""
struct EarthConstantGravity<:Gravity
    hor::Array{T, 1} where T<:Number
end

EarthConstantGravity() = EarthConstantGravity([0, 0, GRAVITY_ACCEL])

calculate_gravity(grav::EarthConstantGravity, pos::Position) = grav
