using FlightMechanics

export Environment, DefaultEnvironment, calculate_environment,
    get_temperature, get_pressure, get_density, get_sound_velocity


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
    wind = calculate_wind(env.wind, get_position(state))
    grav = calculate_gravity(env.grav, state)
    Environment(atmos, wind, grav)
end
