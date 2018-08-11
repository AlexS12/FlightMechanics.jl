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
