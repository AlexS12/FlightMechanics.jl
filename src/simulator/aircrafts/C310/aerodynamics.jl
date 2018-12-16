using Dierckx

using FlightMechanics
using FlightMechanics.Simulator.Models
import FlightMechanics.Simulator.Models: calculate_aerodynamics

export C310Aerodynamics, calculate_aerodynamics


# TODO: CDα, CLα, CYβ... could be included here
struct C310Aerodynamics<:Aerodynamics
    wind_pfm::PointForcesMoments
    wind_coeff_pfm::PointForcesMoments
    body_pfm::PointForcesMoments
    body_coeff_pfm::PointForcesMoments
end


# Drag
#             α (rad)   cDα
tablecDα = [-3.14      0.01  ;
            -1.51      1.    ;
            -0.785     0.707 ;
            -0.0873    0.0041;
            -0.0698    0.0013;
            -0.0524    0.0001;
            -0.0349    0.0003;
            -0.0175    0.0020;
             0.0000    0.0052;
             0.0175    0.0099;
             0.0349    0.0162;
             0.0524    0.0240;
             0.0698    0.0334;
             0.0873    0.0442;
             0.1047    0.0566;
             0.1222    0.0706;
             0.1396    0.0860;
             0.1571    0.0962;
             0.1745    0.1069;
             0.1920    0.1180;
             0.2094    0.1298;
             0.2269    0.1424;
             0.2443    0.1565;
             0.2618    0.1727;
             0.2793    0.1782;
             0.2967    0.1716;
             0.3142    0.1618;
             0.3316    0.1475;
             0.3491    0.1097;
             0.785     0.707 ;
             1.57      1.00  ;
             3.14      0.01  ]
splcDα = Spline1D(tablecDα[:, 1], tablecDα[:, 2], k=1, bc="nearest")

cDα(aero::C310Aerodynamics, α) = splcDα(α)
cD0(aero::C310Aerodynamics) = 0.0280
cDde(aero::C310Aerodynamics, de) = 0.0
cDβ(aero::C310Aerodynamics, β) = 0.1400 * β

# Side
#              β (rad)    cYβ
tablecYβ = [-0.3490     0.2120;
             0.0000     0.0000;
             0.3490    -0.2120]

splcYβ = Spline1D(tablecYβ[:, 1], tablecYβ[:, 2], k=1, bc="nearest")

cYp(aero::C310Aerodynamics, p) = 0.1410 * p
cYr(aero::C310Aerodynamics, r) = 0.3550 * r
cYdr(aero::C310Aerodynamics, dr) = 0.2300 * dr
cYβ(aero::C310Aerodynamics, β) = splcYβ(β)

# Lift
#             α (rad)   cLα
tablecLα = [-3.1416     0.0   ;
            -2.356      0.97  ;
            -1.57       0.0   ;
            -0.785     -0.970 ;
            -0.3000    -1.0100;
            -0.2820    -1.1720;
            -0.2470    -1.0870;
            -0.2120    -0.9480;
            -0.1780    -0.8100;
             0.0000     0.0000;
             0.1040     0.4760;
             0.1390     0.6160;
             0.1740     0.7540;
             0.2090     0.8570;
             0.2440     0.9470;
             0.2790     0.8990;
             0.3000     0.8060;
             0.3140     0.5690;
             0.3490     0.2880;
             0.6        0.904 ;
             0.785      0.97  ;
             1.57       0.0   ;
             2.356     -0.97  ;
             3.14159	 0     ]
splcLα = Spline1D(tablecLα[:, 1], tablecLα[:, 2], k=1, bc="nearest")

#              α (rad)   cLαde
tablecLde = [-1.57       0.0   ;
              0.0000    -0.8100;
              0.0873    -0.9000;
              0.1152    -0.7700;
              1.57       0.0   ]
splcLde = Spline1D(tablecLde[:, 1], tablecLde[:, 2], k=1, bc="nearest")

#          α (rad)   cLααdot
tablecLαdot = [0.0000    5.3000;
          0.0873    4.5000;
          0.1152    4.1000]
splcLαdot = Spline1D(tablecLαdot[:, 1], tablecLαdot[:, 2], k=1, bc="nearest")

#            α (rad)   cLqα
tablecLq = [0.0000    9.7000;
            0.0873    8.8000;
            0.1152    8.4000]
splcLq = Spline1D(tablecLq[:, 1], tablecLq[:, 2], k=1, bc="nearest")

cL0(aero::C310Aerodynamics) = 0.280
cLα(aero::C310Aerodynamics, α) = splcLα(α)
cLde(aero::C310Aerodynamics, de, α) = splcLde(α) * de
cLαdot(aero::C310Aerodynamics, αdot, α) = splcLαdot(α) * αdot
cLq(aero::C310Aerodynamics, q, α) = splcLq(α) * q

# Roll
#             β (rad)    cLβ
tablecLβ = [-0.3490     0.0382;
             0.0000     0.0000;
             0.3490    -0.0382]
# TODO: not a good idea that Spline is created once and again
splcLβ = Spline1D(tablecLβ[:, 1], tablecLβ[:, 2], k=1, bc="nearest")

clp(aero::C310Aerodynamics, p) = -0.7500 * p
clr(aero::C310Aerodynamics, r) = 0.0729 * r
clda(aero::C310Aerodynamics, da) = 0.1720 * da
cldr(aero::C310Aerodynamics, dr) = 0.0192 * dr
clβ(aero::C310Aerodynamics, β) = splcLβ(β)

# Pitch
cm0(aero::C310Aerodynamics) = 0.070
cmα(aero::C310Aerodynamics, α) = -0.989 * α
cmαdot(aero::C310Aerodynamics, αdot) = -12.7000 * αdot
cmq(aero::C310Aerodynamics, q) =  -80.0000 * q
cmde(aero::C310Aerodynamics, de) = -2.2600 * de

# Yaw
cnβ(aero::C310Aerodynamics, β) = 0.1000 * β
cnp(aero::C310Aerodynamics, p) = -0.0257 * p
cnr(aero::C310Aerodynamics, r) = -0.3000 * r
cnda(aero::C310Aerodynamics, da) = -0.0168 * da
cndr(aero::C310Aerodynamics, dr) = -0.1152 * dr

# FORCES
function calculate_aerodynamics(ac::Aircraft, aero::C310Aerodynamics, fcs::FCS,
    aerostate::AeroState, state::State)

    ARP = get_arp(ac)

    qinf = get_qinf(aerostate)
    Sw = get_wing_area(ac)
    b = get_wing_span(ac)
    c = get_chord(ac)

    de = get_value(fcs.de)  # rad
    da = get_value(fcs.da)  # rad
    dr = get_value(fcs.dr)  # rad

    α = get_alpha(aerostate)  # rad
    β = get_beta(aerostate)    # rad
    tas = get_tas(aerostate)  # m/s
    αdot = get_alpha_dot(aerostate)  # rad/s

    c2v = c / (2*tas)
    b2v = b / (2*tas)

    p, q, r = get_body_ang_velocity(state)  # rad/s

    cD = cD0(aero) + cDα(aero, α) + cDde(aero, de) + cDβ(aero, β)
    cY = cYβ(aero, β) + cYdr(aero, dr) +
         b2v * (cYp(aero, p)  + cYr(aero, r))
    cL = cL0(aero) + cLα(aero, α) + cLde(aero, de, α) +
         c2v * (cLαdot(aero, αdot, α) + cLq(aero, q, α))

    cl = clβ(aero, β) + clda(aero, da) + cldr(aero, dr) +
         b2v * (clp(aero, p) + clr(aero, r))
    cm = cm0(aero) + cmα(aero, α) + cmde(aero, de) +
         c2v * (cmαdot(aero, αdot) + cmq(aero, q))
    cn = cnβ(aero, β) + cnda(aero, da) + cndr(aero, dr) +
         b2v * (cnp(aero, p) + cnr(aero, r))

    pfm_wind = PointForcesMoments(ARP,
                                  qinf*Sw*[-cD, cY, -cL],
                                  qinf*Sw*[b*cl, c*cm, b*cn])

    adim_pfm_wind = PointForcesMoments(ARP, [-cD, cY, -cL], [cl, cm, cn])

    pfm_body = rotate(pfm_wind, β, α, 0)
    adim_pfm_body = rotate(adim_pfm_wind, β, α, 0)

    return C310Aerodynamics(pfm_wind, adim_pfm_wind, pfm_body, adim_pfm_body)
end
