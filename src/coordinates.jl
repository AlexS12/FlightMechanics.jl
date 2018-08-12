import .EarthConstants: WGS84


"""
    body2hor(xb, yb, zb, psi, theta, phi)

Transform body coordinates to local horizon.

# Arguments
* `xb, yb, zb`: body coordinates.
* `psi, theta, phi`: Euler angles. Yaw, pitch, roll (rad).
"""
function body2hor(xb, yb, zb, psi, theta, phi)

    s_psi, c_psi = sin(psi), cos(psi)
    s_theta, c_theta = sin(theta), cos(theta)
    s_phi, c_phi = sin(phi), cos(phi)

    xh = (c_theta * c_psi)                         * xb +
         (s_phi * s_theta * c_psi - c_phi * s_psi) * yb +
         (c_phi * s_theta * c_psi + s_phi * s_psi) * zb

    yh = (c_theta * s_psi)                         * xb +
         (s_phi * s_theta * s_psi + c_phi * c_psi) * yb +
         (c_phi * s_theta * s_psi - s_phi * c_psi) * zb

    zh = -s_theta         * xb +
          s_phi * c_theta * yb +
          c_phi * c_theta * zb

    return [xh, yh, zh]
end


"""
    body2hor(xb, yb, zb, q0, q1, q2, q3)

Transform body coordinates to local horizon.

# Arguments
* `xb, yb, zb`: body coordinates.
* `q0, q1, q2, q3`: quaternions.
"""
function body2hor(xb, yb, zb, q0, q1, q2, q3)
    
    q02, q12, q22, q32 = q0*q0, q1*q1, q2*q2, q3*q3
    
    # Linear kinematic equations
    xh = (q02+q12-q22-q32) * xb + 2*(q1*q2 - q0*q3) * yb + 2*(q1*q3 + q0*q2) * zb
    yh = 2*(q1*q2 + q0*q3) * xb + (q02-q12+q22-q32) * yb + 2*(q2*q3 - q0*q1) * zb
    zh = 2*(q1*q3 - q0*q2) * xb + 2*(q2*q3 + q0*q1) * yb + (q02-q12-q22+q32) * zb
    return [xh, yh, zh]
end


"""
    hor2body(xb, yb, zb, q0, q1, q2, q3)

Transform local horizon corrdinates to body coordinates.

# Arguments
* `xh, yh, zh`: local horizon coordinates.
* `q0, q1, q2, q3`: quaternions.
"""
function hor2body(xh, yh, zh, q0, q1, q2, q3)
    
    q02, q12, q22, q32 = q0*q0, q1*q1, q2*q2, q3*q3
    
    # Linear kinematic equations
    xb = (q02+q12-q22-q32) * xh + 2*(q1*q2 + q0*q3) * yh + 2*(q1*q3 - q0*q2) * zh
    yb = 2*(q1*q2 - q0*q3) * xh + (q02-q12+q22-q32) * yh + 2*(q2*q3 + q0*q1) * zh
    zb = 2*(q1*q3 + q0*q2) * xh + 2*(q2*q3 - q0*q1) * yh + (q02-q12-q22+q32) * zh
    return [xb, yb, zb]
end


"""
    hor2body(xh, yh, zh, psi, theta, phi)

Transform local horizon coordinates to body.

# Arguments
* `xh, yh, zh`: local horizon coordinates.
* `psi, theta, phi`: Euler angles. Yaw, pitch, roll (rad).
"""
function hor2body(xh, yh, zh, psi, theta, phi)

    s_psi, c_psi = sin(psi), cos(psi)
    s_theta, c_theta = sin(theta), cos(theta)
    s_phi, c_phi = sin(phi), cos(phi)

    xb =  c_theta * c_psi * xh +
          c_theta * s_psi * yh -
          s_theta         * zh

    yb = (s_phi * s_theta * c_psi - c_psi * s_psi) * xh +
         (s_phi * s_theta * s_psi + c_phi * c_psi) * yh +
         (s_phi * c_theta)                         * zh

    zb = (c_phi * s_theta * c_psi + s_phi * s_psi) * xh +
         (c_phi * s_theta * s_psi - s_phi * c_psi) * yh +
         (c_phi * c_theta)                         * zh

    return [xb, yb, zb]
end


"""
    wind2hor(xw, yw, zw, chi, gamma, mu)

Transform wind coordinates to local horizon.

# Arguments
* `xw, yw, zw`: wind coordinates.
* `chi, gamma, mu`: Velocity angles. Yaw (azimuth), pitch(elevation), roll (rad).
"""
function wind2hor(xw, yw, zw, chi, gamma, mu)

    s_chi, c_chi = sin(chi), cos(chi)
    s_gamma, c_gamma = sin(gamma), cos(gamma)
    s_mu, c_mu = sin(mu), cos(mu)

    xh = (c_gamma * c_chi)                       * xw +
         (s_mu * s_gamma * c_chi - c_mu * s_chi) * yw +
         (c_mu * s_gamma * c_chi + s_mu * s_chi) * zw

    yh = (c_gamma * s_chi)                       * xw +
         (s_mu * s_gamma * s_chi + c_mu * c_chi) * yw +
         (c_mu * s_gamma * s_chi - s_mu * c_chi) * zw

    zh = -s_gamma        * xw +
          s_mu * c_gamma * yw +
          c_mu * c_gamma * zw

    return [xh, yh, zh]
end


"""
    hor2wind(xh, yh, zh, chi, gamma, mu)

Transform local horizon coordinates to wind.

# Arguments
* `xh, yh, zh`: local horizon coordinates.
* `chi, gamma, mu`: Velocity angles. Yaw (azimuth), pitch(elevation), roll (rad).
"""
function hor2wind(xh, yh, zh, chi, gamma, mu)

    s_chi, c_chi = sin(chi), cos(chi)
    s_gamma, c_gamma = sin(gamma), cos(gamma)
    s_mu, c_mu = sin(mu), cos(mu)

    xw =  c_gamma * c_chi * xh +
          c_gamma * s_chi * yh -
          s_gamma         * zh

    yw = (s_mu * s_gamma * c_chi - c_chi * s_chi) * xh +
         (s_mu * s_gamma * s_chi + c_mu  * c_chi) * yh +
         (s_mu * c_gamma)                         * zh

    zw = (c_mu * s_gamma * c_chi + s_mu * s_chi) * xh +
         (c_mu * s_gamma * s_chi - s_mu * c_chi) * yh +
         (c_mu * c_gamma)                        * zh

    return [xw, yw, zw]
end


"""
    body2wind(xb, yb, zb, alpha, beta)

Transform body coordinates to wind.

# Arguments
* `xb, yb, zb`: body coordinates.
* `alpha, beta`: Aerodynamic angles. Angle of attack, angle of side-slip (rad).
"""
function body2wind(xb, yb, zb, alpha, beta)

    s_alpha, c_alpha = sin(alpha), cos(alpha)
    s_beta,  c_beta  = sin(beta),  cos(beta)

    xw =  c_alpha*c_beta * xb + s_beta * yb + s_alpha*c_beta * zb
    yw = -c_alpha*s_beta * xb + c_beta * yb - s_alpha*s_beta * zb
    zw = -s_alpha        * xb +                      c_alpha * zb

    return [xw, yw, zw]
end


"""
    wind2body(xw, yw, zw, alpha, beta)

Transform body coordinates to wind.

# Arguments
* `xw, yw, zw`: wind coordinates.
* `alpha, beta`: Aerodynamic angles. Angle of attack, angle of side-slip (rad).
"""
function wind2body(xw, yw, zw, alpha, beta)

    s_alpha, c_alpha = sin(alpha), cos(alpha)
    s_beta,  c_beta  = sin(beta),  cos(beta)

    xb = c_alpha*c_beta * xw - c_alpha*s_beta * yw - s_alpha * zw
    yb = s_beta         * xw + c_beta         * yw 
    zb = s_alpha*c_beta * xw - s_alpha*s_beta * yw + c_alpha * zw

    return [xb, yb, zb]
end


"""
    ecef2hor(xecef, yecef, zecef, lat, lon)

Transform ECEF (Earth Fixed Earth Centered) coordinates to local horizon
coordinates using geodetic latitude and longitude.

# Arguments
 * `xecef, yecef, zecef`: ECEF coordinates.
 * `lat`: geodetic latitude (rad).
 * `lon`: longitude (rad).
"""
function ecef2hor(xecef, yecef, zecef, lat, lon)
    slat, clat = cos(lat), sin(lat)
    slon, clon = cos(lon), sin(lon)

    xh =  clat * xecef - slat*slon * yecef + slat*clon * zecef
    yh =                 clon      * yecef + slon      * zecef
    zh = -slat * xecef - clat*slon * yecef + clat*clon * zecef

    return [xh, yh, zh]
end


"""
    hor2ecef(xh, yh, zh, lat, lon)

Transform local horizon coordinates to ECEF (Earth Centered Earth Fixed) 
coordinates using geodetic latitude and longitude.

# Arguments
* `xh, yh, zh`: local horizon coordinates.
* `lat`: geodetic latitude (rad).
* `lon`: longitude (rad).

Implementation from:

.. [1] Stevens, B. L., Lewis, F. L., (1992). Aircraft control and simulation:
 dynamics, controls design, and autonomous systems. John Wiley & Sons.
 (page 36, formula 1.4-9)
"""
function hor2ecef(xh, yh, zh, lat, lon)
    slat, clat = cos(lat), sin(lat)
    slon, clon = cos(lon), sin(lon)

    xecef =  clat      * xh +           - slat      * zh
    yecef = -slat*slon * xh + clon * yh - clat*slon * zh
    zecef =  slat*clon * xh + slon * yh + clat*clon * zh

    return [xecef, yecef, zecef]
end


"""
    body2ecef(xb, yb, zb, lat, lon, psi, theta, phi)

Transform body coordinates to ECEF coordinates.

# Arguments
* `xb, yb, zb`: body coordinates.
* `lat`: geodetic latitude (rad).
* `lon`: longitude (rad).
* `psi, theta, phi`: Euler angles. Yaw, pitch, roll (rad).

Implementation from:

.. [1] Stevens, B. L., Lewis, F. L., (1992). Aircraft control and simulation:
 dynamics, controls design, and autonomous systems. John Wiley & Sons.
 (page 36, formula 1.4-9)
"""
function body2ecef(xb, yb, zb, lat, lon, psi, theta, phi)
    xh, yh, zh = body2hor(xb, yb, zb, psi, theta, phi)
    xecef, yecef, zecef = hor2ecef(xh, yh, zh, lat, lon)
    return [xecef, yecef, zecef]
end


"""
    ecef2body(xecef, yecef, zecef, lat, lon, psi, theta, phi)

Transform ECEF coordinates to body coordinates.

# Arguments
* `xecef, yecef, zecef`: ECEF (Earth Centered Earth Fixed) coordinates.
* `lat`: geodetic latitude (rad).
* `lon`: longitude (rad).
* `psi, theta, phi`: Euler angles. Yaw, pitch, roll (rad).
"""
function ecef2body(xecef, yecef, zecef, lat, lon, psi, theta, phi)
    xh, yh, zh = ecef2hor(xecef, yecef, zecef, lat, lon)
    xb, yb, zb = hor2body(xh, yh, zh, psi, theta, phi)
    return [xb, yb, zb]
end


"""
    body2ecef(xb, yb, zb, lat, lon, q0, q1, q2, q3)

Transform body coordinates to ECEF coordinates.

# Arguments
* `xb, yb, zb`: body coordinates.
* `lat`: geodetic latitude (rad).
* `lon`: longitude (rad).
* `q0, q1, q2, q3`: quaternions.
"""
function body2ecef(xb, yb, zb, lat, lon, q0, q1, q2, q3)
    xh, yh, zh = body2hor(xb, yb, zb, q0, q1, q2, q3)
    xecef, yecef, zecef = hor2ecef(xh, yh, zh, lat, lon)
    return [xecef, yecef, zecef]
end


"""
    ecef2body(xecef, yecef, zecef, lat, lon, q0, q1, q2, q3)

Transform ECEF coordinates to body coordinates.

# Arguments
* `xecef, yecef, zecef`: ECEF (Earth Centered Earth Fixed) coordinates.
* `lat`: geodetic latitude (rad).
* `lon`: longitude (rad).
* `q0, q1, q2, q3`: quaternions.
"""
function ecef2body(xecef, yecef, zecef, lat, lon, q0, q1, q2, q3)
    xh, yh, zh = ecef2hor(xecef, yecef, zecef, lat, lon)
    xb, yb, zb = hor2body(xh, yh, zh, q0, q1, q2, q3)
    return [xb, yb, zb]
end


"""
    euler2quaternion(psi, theta, phi)

Transform Euler angles orientation to quaternion orientation.

# Arguments
* `psi, theta, phi`: Euler angles. Yaw, pitch, roll (rad).

Implementation from:
.. [1] Zipfel, P. H. (2007). Modeling and simulation of aerospace vehicle
 dynamics. American Institute of Aeronautics and Astronautics.
 (page 126, formula 4.78)
"""
function euler2quaternion(psi, theta, phi)

    s_psi2, c_psi2 = sin(psi/2.0), cos(psi/2.0)
    s_theta2, c_theta2 = sin(theta/2.0), cos(theta/2.0)
    s_phi2, c_phi2 = sin(phi/2.0), cos(phi/2.0)

    q0 = c_psi2*c_theta2*c_phi2 + s_psi2*s_theta2*s_phi2
    q1 = c_psi2*c_theta2*s_phi2 - s_psi2*s_theta2*c_phi2
    q2 = c_psi2*s_theta2*c_phi2 + s_psi2*c_theta2*s_phi2
    q3 = s_psi2*c_theta2*c_phi2 - c_psi2*s_theta2*s_phi2
    
    return [q0, q1, q2, q3]
end


"""
    quaternion2euler(q0, q1, q2, q3)

Transform quaternion orientation to Euler angles orientation.

#Arguments
`q0, q1, q2, q3`: quaternions.

Implementation from:
.. [1] Zipfel, P. H. (2007). Modeling and simulation of aerospace vehicle
 dynamics. American Institute of Aeronautics and Astronautics.
 (page 127, formula 4.82)
"""
function quaternion2euler(q0, q1, q2, q3)
    psi = atan2(2 * (q1*q2 + q0*q3), q0*q0 + q1*q1 - q2*q2 - q3*q3)
    theta = asin(-2 * (q1*q3 - q0*q2))
    phi = atan2(2 * (q2*q3 + q0 * q1), q0*q0 - q1*q1 - q2*q2 + q3*q3)

    return [psi, theta, phi]
end


"""
    llh2ecef(lat, lon, height; ellipsoid=WGS84)

Transform geodetic latitude, longitude and ellipsoidal height to ECEF for the
given ellipsoid (default ellipsoid is WGS84)

Implementation from:
.. [1] Rogers, R. M. (2007). Applied mathematics in integrated navigation
 systems. American Institute of Aeronautics and Astronautics.
 (Page 75, equations 4.20, 4.21, 4.22)
"""
function llh2ecef(lat, lon, height; ellipsoid=WGS84)
    f = ellipsoid.f
    a = ellipsoid.a
    e2 = ellipsoid.e2

    var = a / sqrt(1.0 - e2 * sin(lat)*sin(lat))

    px = (var + height) * cos(lat)*cos(lon)
    py = (var + height) * cos(lat)*sin(lon)
    pz = (var * (1.0 - e2) + height)* sin(lat)

    return [px, py, pz]
end


"""
    ecef2llh(xecef, yecef, zecef; ellipsoid=WGS84)

Transform ECEF coordinates to geodetic latitude, longitude and ellipsoidal 
height for the given ellipsoid (default ellipsoid is WGS84)

Implementation from:
.. [1] Bowring, B. R. (1976). Transformation from spatial to geographical 
 coordinates. Survey review, 23(181), 323-327.
.. [2] Bowring, B. R. (1985). The accuracy of geodetic latitude and height
 equations. Survey Review, 28(218), 202-206.

Notes
-----
* The transformation is direct without iterations as [1] introduced the need to
iterate for near Earth positions.
* [2] is an updated of incresed accuracy of [1]. The former is used for
in this implementation although the latter implementation is commented in the 
code.
* Model becomes unstable if latitude is close to 90º. An alternative equation
can be found in [2] equation (16) but has not been implemented.
"""
function ecef2llh(xecef, yecef, zecef; ellipsoid=WGS84)

    x, y, z = xecef, yecef, zecef
    e = sqrt(ellipsoid.e);
    e2 = ellipsoid.e2
    ϵ2 = ellipsoid.ϵ2
    a = ellipsoid.a
    b = ellipsoid.b

    p = sqrt(x*x + y*y)
    R = sqrt(p*p + z*z)
    θ = atan2(z, p)

    # [1] equation (1) does not change in [2]
    lon = atan2(y, x)

    # u -> geographical latitude
    # [1] below equation (4) and [2] equation (6)
    # This lead to errors in latitud with maximum value 0.0018"
    #u = atan(a/b * z/x)

    # [2] equation (17) If the latitude is also required to be very accurate
    # for outer-space positions then the value of u for (6) should be obtained
    # from:
    u = atan( b*z / (a*p) * (1 + ϵ2 * b / R) )

    # [1] equation (4)
    #lat = atan((z + ϵ2 * b * sin(u)^3) / (x - e2 * a * cos(u)^3))

    # [2] equation (18)
    lat = atan( (z + ϵ2 * b * sin(u)^3) / (p - e2 * a * cos(u)^3) )
    
    # [2] after equation (1)
    v = a / sqrt(1.0 - e2*sin(lat)^2)
    # [2] equation (7)
    # height = p*cos(lat) + z*sin(lat) - a*a / v
    # equivalent to [2] equation (8) 
    height = R * cos(lat - θ) - a*a / v
    # which is insensitive to the error in latitude calculation (The influence
    # is of order 2 [2] equation (12))
    return [lat, lon, height]
end