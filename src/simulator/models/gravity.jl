using FlightMechanics

export EarthConstantGravity, calculate_gravity,
    get_gravity_accel, get_grav_body_vector, get_grav_body_versor


abstract type Gravity end

get_gravity_accel(grav::Gravity) = grav.value
get_grav_body_vector(grav::Gravity) = grav.body_vector
get_grav_body_versor(grav::Gravity) = grav.body_vector / grav.value


"""
    EarthConstantGravity(value, body_vector)

Earth Constant gravity model with value 9.80665 m/s² and oriented towards
local horizon Z axis.

# Constructors
    EarthConstantGravity(value, body_vector)
    EarthConstantGravity()
"""
struct EarthConstantGravity<:Gravity
    value::Number  # m/s²
    body_vector::Array{T, 1} where T<:Number
end

EarthConstantGravity() = EarthConstantGravity(GRAVITY_ACCEL, [0, 0, GRAVITY_ACCEL])


function EarthConstantGravity(state::State)
    grav = EarthConstantGravity()
    calculate_gravity(grav, state)
end

function calculate_gravity(grav::EarthConstantGravity, state::State)
    g = GRAVITY_ACCEL
    EarthConstantGravity(g, hor2body(0, 0, g, get_euler_angles(state)...))
end
