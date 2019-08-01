"""
    Environment(atmos::Atmosphere, wind::Wind, grav::Gravity)

Environment information composed of atmosphere, wind and gravity.

# Constructors
    Environment(atmos::Atmosphere, wind::Wind, grav::Gravity)
    Environment(pos::Position; atmos="ISA1978", wind="NoWind", grav="const")
"""
struct Environment
    atmos::Atmosphere
    wind::Wind
    grav::Gravity
end


"""
    Environment(pos::Position; atmos="ISA1978", wind="NoWind", grav="const")

Create an environment for the given position. Models can be chosen for:
    - atmos: atmosphere model.
        - ISA1978: default. International Standard Atmosphere 1978.
        - ISAF16: International Standard Atmosphere 1978 in F16 Stevens model.
    - wind: wind model.
        - NoWind: default.
    - grav: gravity model.
        - const: constant vertical gravity.
"""
function Environment(pos::Position; atmos="ISA1978", wind="NoWind", grav="const")
    # atm
    atmos = uppercase(atmos)
    if atmos == "ISA1978"
        atm_model = AtmosphereISA(pos)
    elseif atmos == "ISAF16"
        atm_model = AtmosphereF16(pos)
    else
        error("Unknown atmospheric model ($atmos).")
    end

    # wind
    wind = uppercase(wind)
    if wind == "NOWIND"
        wind_model = ConstantWind()
    else
        error("Unknown wind model ($wind).")
    end

    # Gravity
    grav = uppercase(grav)
    if grav == "CONST"
        grav_model = EarthConstantGravity()
    else
        error("Unknown gravity model ($grav).")
    end

    Environment(atm_model, wind_model, grav_model)
end


get_atmos(env::Environment) = env.atmos
get_temperature(env::Environment) = get_temperature(env.atmos)
get_pressure(env::Environment) = get_pressure(env.atmos)
get_density(env::Environment) = get_density(env.atmos)
get_sound_velocity(env::Environment) = get_sound_velocity(env.atmos)

get_wind(env::Environment) = env.wind
get_wind_dir_int_ver(env::Environment) = get_wind_dir_int_ver(env.wind)
get_wind_NED(env::Environment) = get_wind_NED(env.wind)
get_wind_body(env::Environment, att::Attitude) = get_wind_body(env.wind, att)
get_wind_direction(env::Environment) = get_wind_direction(env.wind)
get_wind_intensity(env::Environment) = get_wind_intensity(env.wind)
get_wind_vertical(env::Environment) = get_wind_vertical(env.wind)

get_gravity(env::Environment) = env.grav
get_gravity_horizon(env::Environment) = get_gravity_horizon(env.grav)
get_gravity_accel(env::Environment) = get_gravity_accel(env.grav)
get_gravity_body(env::Environment, att::Attitude) = get_gravity_body(env.grav, att)


function calculate_environment(env::Environment, pos::Position)
    atmos = calculate_atmosphere(env.atmos, pos)
    wind = calculate_wind(env.wind, pos)
    grav = calculate_gravity(env.grav, pos)
    Environment(atmos, wind, grav)
end
