# -------- WIND --------
"""
    Wind

Wind information at a given placement.
"""
abstract type Wind end


"""
    get_wind_dir_int_ver(wind::Wind)

- direction: wind blowing from that direction [rad]
- intensity: wind speed [m/s]
- vertical: vertical wind speed [m/s] positive blowing upwards.
"""
get_wind_dir_int_ver(wind::Wind) = [get_wind_direction(wind), get_wind_intensity(wind), get_wind_vertical(wind)]

"""
Wind blowing from this direction [rad]
"""
get_wind_direction(wind::Wind) = wind.direction

"""
Wind magnitude ≥ 0  [m/s]
"""
get_wind_intensity(wind::Wind) = wind.intensity

"""
Wind vertical magnitude [m/s]. Possitive blowing up.
"""
get_wind_vertical(wind::Wind) = wind.vertical

"""
    get_wind_NED(wind::Wind)

Express wind in local horizon axis [N, E, D]. Must be interpreted as wind coming
from north, east and down [m/s].
"""
function get_wind_NED(wind::Wind)
    intensity = get_wind_intensity(wind)
    direction = get_wind_direction(wind)
    # coming from
    wind_N = intensity * cos(direction)
    wind_E = intensity * sin(direction)
    wind_D = get_wind_vertical(wind)
    return [wind_N, wind_E, wind_D]
end

"""
    get_wind_body(wind::Wind, att::Attitude)

Express wind in body axis given the `Attitude`. Must be interpreted as wind coming
from forward, right and down directions [m/s]
"""
function get_wind_body(wind::Wind, att::Attitude)
    wind_ned = get_wind_NED(wind)
    hor2body(wind_ned..., get_quaternions(att)...)
end


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

calculate_wind(wind::ConstantWind, pos::Position) = wind
