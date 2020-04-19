using FlightMechanics


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
PositionLLH(lat, lon, h, xe=0., ye=0.) = Position([lat, lon, h], [xe, ye, -h], llh2ecef(lat, lon, h))
PositionECEF(x, y, z, xe=0., ye=0.) = Position(ecef2llh(x, y, z), [xe, ye, -ecef2llh(x, y, z)[3]], [x, y, z])
PositionEarth(x, y, z, lat=0., lon=0.) = Position([lat, lon, -z], [x, y, z], llh2ecef(lat, lon, -z))
Position() = Position([0, 0, 0], [0, 0, 0], llh2ecef(0, 0, 0))

get_llh(pos::Position) = pos.llh
get_xyz_earth(pos::Position) = pos.xyz_earth
get_xyz_ecef(pos::Position) = pos.xyz_ecef
get_height(pos::Position) = pos.llh[3]


function isapprox(x::Position, y::Position; rtol=1e-8, atol=0.0, nans=false)

    result = all([
        isapprox(x.llh, y.llh, rtol=rtol, atol=atol, nans=nans),
        isapprox(x.xyz_earth, y.xyz_earth, rtol=rtol, atol=atol, nans=nans),
        isapprox(x.xyz_ecef, y.xyz_ecef, rtol=rtol, atol=atol, nans=nans),
    ])

    return result
end

# -------------- POSITIONS --------------
struct ECEFPosition
    x :: Number  # m
    y :: Number  # m
    z :: Number  # m
end


struct LLHPosition
    lat :: Number  # rad
    lon :: Number  # rad
    height :: Number  # m
    ellipsoid :: Ellipsoid
end

LLHPosition(ellipsoid) = LLHPosition(NaN, NaN, NaN, ellipsoid)
LLHPosition(lat, lon, h) = LLHPosition(lat, lon, h, WGS84)
LLHPosition() = LLHPosition(NaN, NaN, NaN)


struct EarthPosition
    x :: Number  # m
    y :: Number  # m
    z :: Number  # m
    ref_point :: T where T<:Union{LLHPosition}
end

EarthPosition(x, y, z) = EarthPosition(x, y, z, LLHPosition(0, 0, 0))
EarthPosition(ref_point) = EarthPosition(NaN, NaN, NaN, ref_point)


# -------------- CONVERTERS --------------
convert(llh::LLHPosition, pos::ECEFPosition) = LLHPosition(
    ecef2llh(
        pos.x,
        pos.y,
        pos.z,
        ellipsoid=llh.ellipsoid,
    )...
)


convert(::Type{ECEFPosition}, pos::LLHPosition) = ECEFPosition(
    llh2ecef(
        pos.lat,
        pos.lon,
        pos.height,
        ellipsoid=pos.ellipsoid
    )...
)


function convert(earth::EarthPosition, ecef::ECEFPosition)
    ref_point_ecef = convert(ECEFPosition, earth.ref_point)

    dx = ecef.x - ref_point_ecef.x
    dy = ecef.y - ref_point_ecef.y
    dz = ecef.z - ref_point_ecef.z

    dx, dy, dz = ecef2hor(dx, dy, dz, earth.ref_point.lat, earth.ref_point.lon)

    return EarthPosition(dx, dy, dz, earth.ref_point)
end


function convert(::Type{ECEFPosition}, earth::EarthPosition)
    ref_point_ecef = convert(ECEFPosition, earth.ref_point)

    dx, dy, dz = hor2ecef(
        earth.x,
        earth.y,
        earth.z,
        earth.ref_point.lat,
        earth.ref_point.lon
    )

    return ECEFPosition(
        ref_point_ecef.x + dx,
        ref_point_ecef.y + dy,
        ref_point_ecef.z + dz
    )
end


function convert(::Type{LLHPosition}, earth::EarthPosition)
    ecef_pos = convert(ECEFPosition, earth)
    return convert(earth.ref_point, ecef_pos)
end


function convert(llh_to::LLHPosition, llh_from::LLHPosition)
    ecef_from = convert(ECEFPosition, llh_from)
    llh_to = convert(llh_to, ecef_from)
    return llh_to
end


function convert(earth::EarthPosition, llh::LLHPosition)
    ecef = convert(ECEFPosition, llh)
    earth = convert(earth, ecef)
    return earth
end


# -------------- COMPARISON --------------
function isapprox(x::ECEFPosition, y::ECEFPosition; rtol=1e-8, atol=0.0, nans=false)
    return isapprox([x.x, x.y, x.z], [y.x, y.y, y.z]; rtol=rtol, atol=atol, nans=nans)
end


function isapprox(x::LLHPosition, y::LLHPosition; rtol=1e-8, atol=0.0, nans=false)
    r = all([
        isapprox(x.lat, y.lat; rtol=rtol, atol=atol, nans=nans),
        isapprox(x.lon, y.lon; rtol=rtol, atol=atol, nans=nans),
        isapprox(x.height, y.height; rtol=rtol, atol=atol, nans=nans),
        x.ellipsoid == y.ellipsoid,
    ])
    return r
end


function isapprox(x::EarthPosition, y::EarthPosition; rtol=1e-8, atol=0.0, nans=false)
    r = all([
        isapprox(x.x, y.x; rtol=rtol, atol=atol, nans=nans),
        isapprox(x.y, y.y; rtol=rtol, atol=atol, nans=nans),
        isapprox(x.z, y.z; rtol=rtol, atol=atol, nans=nans),
        isapprox(x.ref_point, y.ref_point; rtol=rtol, atol=atol, nans=nans),
    ])
    return r
end