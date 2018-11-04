using FlightMechanics
using FlightMechanics.Simulator.Models


export AtmosphereISA, calculate_atmosphere
export ConstantWind, calculate_wind
export EarthConstantGravity, calculate_gravity
export Environment, calculate_environment


# -------- ATMOSPHERE --------
abstract type Atmosphere end

struct AtmosphereISA<:Atmosphere
    temperature::Number  # K
    pressure::Number  # Pa
    density::Number  # kg/m³
    sound_velocity::Number  # m/s
end

# Sea level
AtmosphereISA() = AtmosphereISA(T0, P0, RHO0, A0)

function calculate_atmosphere(atmos::AtmosphereISA, state::State)
    AtmosphereISA(atmosphere_isa(get_height(state))...)
end

# -------- WIND --------
abstract type Wind end

struct ConstantWind<:Wind
    wind_direction::Number  # coming from [rad]
    wind_intensity::Number  # positive blowing to ac [m/s]
    wind_vertical::Number   # positive blowing to ac [m/s]
end

ConstantWind() = ConstantWind(0, 0, 0)

calculate_wind(wind::ConstantWind, state::State) = wind

# -------- GRAVITY --------
abstract type Gravity end

struct EarthConstantGravity<:Gravity
    value::Number  # m/s²
end

EarthConstantGravity() = EarthConstantGravity(GRAVITY_ACCEL)

calculate_gravity(grav::EarthConstantGravity, state::State) = grav

# -------- ENVRIONMENT --------
struct Environment
    atmos::Atmosphere
    wind::Wind
    grav::Gravity
end

DefaultEnvironment = Environment(AtmosphereISA(), ConstantWind(), EarthConstantGravity())

function calculate_environment(env::Environment, state::State)
    atmos = calculate_atmosphere(env.atmos, state)
    wind = calculate_wind(env.wind, state)
    grav = calculate_gravity(env.grav, state)
    Environment(atmos, wind, grav)
end
