using FlightMechanics


# -------------- POSITIONS --------------
abstract type Position end


struct ECEFPosition <: Position
    x :: Number  # m
    y :: Number  # m
    z :: Number  # m
end


show(io::IO, pos::ECEFPosition) = print(
    io,
    "ECEF Position: x=$(pos.x) m, y=$(pos.y) m, z=$(pos.z) m",
    )


struct LLHPosition <: Position
    lat :: Number  # rad
    lon :: Number  # rad
    height :: Number  # m
    ellipsoid :: Ellipsoid
end

show(io::IO, pos::LLHPosition) = print(
    io,
    "LLH $(pos.ellipsoid.name): lat=$(rad2deg(pos.lat))º, lon=$(rad2deg(pos.lon))º, h=$(pos.height) m",
    )


LLHPosition(ellipsoid) = LLHPosition(NaN, NaN, NaN, ellipsoid)
LLHPosition(lat, lon, h) = LLHPosition(lat, lon, h, WGS84)
LLHPosition() = LLHPosition(NaN, NaN, NaN)


struct EarthPosition <: Position
    x :: Number  # m
    y :: Number  # m
    z :: Number  # m
    ref_point :: T where T<:Union{LLHPosition}
end


show(io::IO, pos::EarthPosition) = print(
    io,
    "Earth Position (wrt $(pos.ref_point)): x=$(pos.x) m, y=$(pos.y) m, z=$(pos.z) m"
    )


EarthPosition(x, y, z) = EarthPosition(x, y, z, LLHPosition(0, 0, 0))
EarthPosition(ref_point) = EarthPosition(NaN, NaN, NaN, ref_point)


# -------------- GETTERS --------------
function get_llh(pos::Position; ellipsoid=WGS84)
    llh = convert(LLHPosition(ellipsoid), pos)
    return [llh.lat, llh.lon, llh.height]
end


function get_xyz_earth(pos::Position; ref_point=LLHPosition(0.0, 0.0, 0.0, WGS84))
    earth = convert(EarthPosition(ref_point), pos)
    return [earth.x, earth.y, earth.z]
end


function get_xyz_ecef(pos::Position)
    ecef = convert(ECEFPosition, pos)
    return [ecef.x, ecef.y, ecef.z]
end


get_height(pos::EarthPosition) = -pos.z
get_height(pos::LLHPosition) = pos.height
get_height(pos::ECEFPosition; ellipsoid=WGS84) = get_llh(pos, ellipsoid=ellipsoid)[3]

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


function convert(llh::LLHPosition, earth::EarthPosition)
    ecef_pos = convert(ECEFPosition, earth)
    return convert(llh, ecef_pos)
end

# TODO: not tested
function convert(llh_to::LLHPosition, llh_from::LLHPosition)
    ecef_from = convert(ECEFPosition, llh_from)
    llh_to = convert(llh_to, ecef_from)
    return llh_to
end


# TODO: not tested
function convert(earth_to::EarthPosition, earth_from::EarthPosition)
    ecef_earth_from = convert(ECEFPosition, earth_from)
    earth_to = convert(earth_to, ecef_earth_from)
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