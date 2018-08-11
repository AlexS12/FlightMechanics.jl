"""
    body2hor(xb, yb, zb, chi, theta, phi)

Transform body coordinates to local horizon.

# Arguments
* `xb, yb, zb`: body coordinates.
* `chi, theta, phi`: Euler angles. Yaw, pitch, roll (rad).
"""
function body2hor(xb, yb, zb, chi, theta, phi)

    s_chi, c_chi = sin(chi), cos(chi)
    s_theta, c_theta = sin(theta), cos(theta)
    s_phi, c_phi = sin(phi), cos(phi)

    xh = (c_theta * c_chi)                         * xb +
         (s_phi * s_theta * c_chi - c_phi * s_chi) * yb +
         (c_phi * s_theta * c_chi + s_phi * s_chi) * zb

    yh = (c_theta * s_chi)                         * xb +
         (s_phi * s_theta * s_chi + c_phi * c_chi) * yb +
         (c_phi * s_theta * s_chi - s_phi * c_chi) * zb

    zh = -s_theta         * xb +
          s_phi * c_theta * yb +
          c_phi * c_theta * zb

    return [xh, yh, zh]
end


"""
    hor2body(xh, yh, zh, chi, theta, phi)

Transform local horizon coordinates to body.

# Arguments
* `xh, yh, zh`: local horizon coordinates.
* `chi, theta, phi`: Euler angles. Yaw, pitch, roll (rad).
"""
function hor2body(xh, yh, zh, chi, theta, phi)

    s_chi, c_chi = sin(chi), cos(chi)
    s_theta, c_theta = sin(theta), cos(theta)
    s_phi, c_phi = sin(phi), cos(phi)

    xb =  c_theta * c_chi * xh +
          c_theta * s_chi * yh -
          s_theta         * zh

    yb = (s_phi * s_theta * c_chi - c_chi * s_chi) * xh +
         (s_phi * s_theta * s_chi + c_phi * c_chi) * yh +
         (s_phi * c_theta)                         * zh

    zb = (c_phi * s_theta * c_chi + s_phi * s_chi) * xh +
         (c_phi * s_theta * s_chi - s_phi * c_chi) * yh +
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
"""
function hor2ecef(xh, yh, zh, lat, lon)
    slat, clat = cos(lat), sin(lat)
    slon, clon = cos(lon), sin(lon)

    xecef =  clat      * xh +           - slat      * zh
    yecef = -slat*slon * xh + clon * yh - clat*slon * zh
    zecef =  slat*clon * xh + slon * yh + clat*clon * zh

    return [xecef, yecef, zecef]  
end
