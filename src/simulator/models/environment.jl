using FlightMechanics


export ConstantWind, calculate_wind
export EarthConstantGravity, calculate_gravity
export Environment, DefaultEnvironment, calculate_environment,
    get_temperature, get_pressure, get_density, get_sound_velocity,
    get_wind, get_wind_NED, get_wind_direction, get_wind_intensity, get_wind_vertical,
    get_gravity_accel, get_grav_body_vector, get_grav_body_versor


# -------- WIND --------
"""
    Wind

Wind information at a given placement.
"""
abstract type Wind end


"""
    ConstantWind(direction, intensity, vertical)

Constant wind structure:
  - direction: wind blowing from that direction [rad]
  - intensity: wind speed [m/s]
  - vertical: vertical wind speed [m/s] positive blowing upwards.

 # Constructors

    ConstantWind(): No wind.
"""
struct ConstantWind<:Wind
    direction::Number  # coming from [rad]
    intensity::Number  # positive blowing to ac [m/s]
    vertical::Number   # positive blowing to ac [m/s]
end

ConstantWind() = ConstantWind(0, 0, 0)

get_wind(wind::Wind) = [wind.direction, wind.intensity, wind.vertical]
get_direction(wind::Wind) = wind.direction
get_intensity(wind::Wind) = wind.intensity
get_vertical(wind::Wind) = wind.vertical


"""
    get_wind_NED(wind::Wind)

Express wind in local horizon axis [N, E, D].
"""
function get_wind_NED(wind::Wind)
    # coming from
    wind_N = wind.intensity * cos(wind.direction + π)
    wind_E = wind.intensity * sin(wind.direction + π)
    wind_D = wind.vertical
    return [wind_N, wind_E, wind_D]
end

"""
    get_wind_body(wind::Wind, state::State)

Express wind body axis [N, E, D] given the `Attitude` stored in `State`.
"""
function get_wind_body(wind::Wind, state::State)
    wind_ned = get_wind_NED(wind)
    hor2body(wind_ned..., get_quaternion(state)...)
end

calculate_wind(wind::ConstantWind, state::State) = wind

# -------- GRAVITY --------
abstract type Gravity end

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

get_gravity_accel(grav::Gravity) = grav.value
get_grav_body_vector(grav::Gravity) = grav.body_vector
get_grav_body_versor(grav::Gravity) = grav.body_vector / grav.value

function calculate_gravity(grav::EarthConstantGravity, state::State)
    g = GRAVITY_ACCEL
    EarthConstantGravity(g, hor2body(0, 0, g, get_euler_angles(state)...))
end

# -------- ENVRIONMENT --------
"""
    Environment(atmos::Atmosphere, wind::Wind, grav::Gravity)

Environment information composed of atmosphere, wind and gravity.

# Constructors
    DefatultEnvironment(): ISA atomosphere, Constant zero wind, and Constant gravity.
"""
# TODO: every environment must be initialized with a state
struct Environment
    atmos::Atmosphere
    wind::Wind
    grav::Gravity
end

DefaultEnvironment() = Environment(AtmosphereISA(), ConstantWind(), EarthConstantGravity())

get_temperature(env::Environment) = get_temperature(env.atmos)
get_pressure(env::Environment) = get_pressure(env.atmos)
get_density(env::Environment) = get_density(env.atmos)
get_sound_velocity(env::Environment) = get_sound_velocity(env.atmos)

get_wind(env::Environment) = get_wind(env.wind)
get_wind_NED(env::Environment) = get_wind_NED(env.wind)
get_wind_body(env::Environment, state::State) = get_wind_body(env.wind, state)
get_wind_direction(env::Environment) = get_wind_direction(env.wind)
get_wind_intensity(env::Environment) = get_wind_intensity(env.wind)
get_wind_vertical(env::Environment) = get_wind_vertical(env.wind)

get_gravity_accel(env::Environment) = get_gravity_accel(env.grav)
get_grav_body_vector(env::Environment) = get_grav_body_vector(env.grav)
get_grav_body_versor(env::Environment) = get_grav_body_versor(env.grav)


function calculate_environment(env::Environment, state::State)
    atmos = calculate_atmosphere(env.atmos, state)
    wind = calculate_wind(env.wind, state)
    grav = calculate_gravity(env.grav, state)
    Environment(atmos, wind, grav)
end
