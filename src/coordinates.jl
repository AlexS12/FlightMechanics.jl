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
