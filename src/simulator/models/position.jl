using FlightMechanics

export State,
       Position, PositionLLH, PositionEarth, PositionECEF,
       get_position, get_llh, get_xyz_earth, get_xyz_ecef, get_height


"""
    Position(llh, xzy_earth, xyz_ecef)

Position information:
    - llh (latitude, longitude, height) [rad, rad, m]
    - Earth (x, y, z) [m]
    - ECEF (x, y, z) [m]

# Constructors
    PositionLLH(lat, lon, h, xe=0., ye=0., ze=0.)
    PositionECEF(x, y, z, xe=0., ye=0., ze=0.)
    PositionEarth(x, y, z, lat=0., lon=0., h=0.)
"""
struct Position
    # latitude [rad], longitude [rad], height [m]
    llh::Array{T, 1} where T<:Number
    # x, y, z [m] Earth
    xyz_earth::Array{T, 1} where T<:Number
    # x, y, z [m] ECEF
    xyz_ecef::Array{T, 1} where T<:Number
end

# Constructors
PositionLLH(lat, lon, h, xe=0., ye=0., ze=0.) = Position([lat, lon, h], [xe, ye, ze], llh2ecef(lat, lon, h))
PositionECEF(x, y, z, xe=0., ye=0., ze=0.) = Position(ecef2llh(x, y, z), [xe, ye, ze], [x, y, z])
PositionEarth(x, y, z, lat=0., lon=0., h=0.) = Position([lat, lon, h], [x, y, z], llh2ecef(lat, lon, h))
Position() = Position([0, 0, 0], [0, 0, 0], llh2ecef(0, 0, 0))

get_llh(pos::Position) = pos.llh
get_xyz_earth(pos::Position) = pos.xyz_earth
get_xyz_ecef(pos::Position) = pos.xyz_ecef
get_height(pos::Position) = pos.llh[3]
