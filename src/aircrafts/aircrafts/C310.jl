using FlightMechanics
using FlightMechanics.Models
using Dierckx

##----------------------------------------------------------------------------------------------------
## imports

# Aerodynamics
import FlightMechanics.Models:
    calculate_aerodynamics

# Propulsion
import FlightMechanics.Models:
    get_pfm, get_cj, get_power, get_efficiency, get_tanks,
    get_engine_position, get_engine_orientation, get_engine_gyro_effects,
    calculate_engine

# Aircraft Model
import FlightMechanics.Models:
    get_name, get_wing_area, get_wing_span, get_chord, get_arp,
    get_empty_mass_props, get_payload_mass_props


##----------------------------------------------------------------------------------------------------
## exports

# Aerodynamics
export C310Aerodynamics,
    calculate_aerodynamics     

# Propulsion
export C310Engine, C310EngineLeft, C310EngineRight,
    get_pfm, get_cj, get_power, get_efficiency, get_tanks,
    get_engine_position, get_engine_orientation,
    calculate_engine

# FCS
export C310FCS,
    set_stick_lon!, set_stick_lat!, set_pedals!,
    set_thtl1!, set_thtl2!, set_thtl!,
    set_controls_trimmer!, get_controls_trimmer,
    get_controls_ranges_trimmer

# Aircraft Model
export C310,
    get_name,
    get_payload_mass_props,
    get_empty_mass_props,
    get_wing_area,
    get_wing_span,
    get_chord,
    get_arp

    
##----------------------------------------------------------------------------------------------------
## structs

# Aerodynamics
struct C310Aerodynamics<:Aerodynamics       # TODO: CDα, CLα, CYβ... could be included here
    wind_pfm::PointForcesMoments
    wind_coeff_pfm::PointForcesMoments
    body_pfm::PointForcesMoments
    body_coeff_pfm::PointForcesMoments
end

# Propulsion
abstract type C310Engine<:Engine end

struct C310EngineLeft<:C310Engine
    pfm::PointForcesMoments
    cj::Number
    power::Number
    efficiency::Number
    tanks::Array{RigidSolid, 1}
    h::Array{T, 1} where T<:Number  # angular momentum [kg·m²/s]
end

struct C310EngineRight<:C310Engine
    pfm::PointForcesMoments
    cj::Number
    power::Number
    efficiency::Number
    tanks::Array{RigidSolid, 1}
    h::Array{T, 1} where T<:Number  # angular momentum [kg·m²/s]
end

# FCS
struct C310FCS<:FCS
    # Cabin controls
    stick_longitudinal::RangeControl
    stick_lateral::RangeControl
    pedals::RangeControl
    thtl::RangeControl

    # Aerodynamic surfaces
    de::RangeControl
    da::RangeControl
    dr::RangeControl
    # Engine
    t1::RangeControl
    t2::RangeControl
end

# Aircraft Model
struct C310<:Aircraft
    mass_props::RigidSolid
    pfm::PointForcesMoments

    aerodynamics::C310Aerodynamics
    propulsion::Propulsion
end


##----------------------------------------------------------------------------------------------------
## Aerodynamics

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

    aerodynamics_from_wind_total(pfm_wind, α, β, qinf, Sw, b, c, aero)
end


##----------------------------------------------------------------------------------------------------
## Propulsion

function C310EngineLeft()
    C310EngineLeft(
        PointForcesMoments(zeros(3), zeros(3), zeros(3)),
        0,
        0,
        0,
        [PointMass(225 * LB2KG, [35, -209.8, 28.3] .* IN2M),
         PointMass(95 * LB2KG, [35, -41.6, 11.7] .* IN2M)],
        [0.0, 0.0, 0.0]
        )
end

function C310EngineRight()
    C310EngineRight(
        PointForcesMoments(zeros(3), zeros(3), zeros(3)),
        0,
        0,
        0,
        [PointMass(225 * LB2KG, [35, 209.8, 28.3] .* IN2M),
         PointMass(95 * LB2KG, [35, 41.6, 11.7] .* IN2M)],
        [0.0, 0.0, 0.0]
        )
end

#Getters
get_pfm(eng::C310Engine) = eng.pfm
get_cj(eng::C310Engine) = eng.cj
get_power(eng::C310Engine) = eng.power
get_efficiency(eng::C310Engine) = eng.efficiency
get_tanks(eng::C310Engine) = eng.tanks

get_engine_position(prop::C310EngineLeft) = [-27.5, -70, 15.5] .* IN2M
get_engine_position(prop::C310EngineRight) = [-27.5, 70, 15.5] .* IN2M
get_engine_orientation(prop::C310Engine) = [0, 0, 0] .* DEG2RAD


function calculate_engine_power(eng::C310Engine, fcs::FCS,
                                aerostate::AeroState, state::State)
   # Max power at sea level
   Pmax0 = 260 * HP2WAT
   # Power assumed proportional to thrust lever
   P0 = Pmax0 * get_thrust_setting(eng, fcs)
   # Assume supercharged engine, no variation of power with altitude until crtic altitude
   Pm = P0 * 1.0
   # Calculate fuel consumption also
   cj = 0.0
   return Pm, cj
end

function calculate_propeller_thrust(eng::C310Engine, fcs::FCS,
    aerostate::AeroState, state::State, Pm::Number)
   # Assume constant propulsive efficiency
   ηp = 0.75
   # Calculate thrust
   thrust = ηp * Pm / get_tas(aerostate)
   return thrust, ηp
end


function calculate_engine(eng::C310Engine, fcs::FCS, aerostate::AeroState,
                          state::State; consume_fuel=false)
    Pm, cj = calculate_engine_power(eng, fcs, aerostate, state)
    thrust, ηp = calculate_propeller_thrust(eng, fcs, aerostate, state, Pm)

    if consume_fuel==true
        error("fuel consumption not implemented")
    else
        tanks = get_tanks(eng)
    end

    # TODO: Torque calcualtion is missing
    pfm = PointForcesMoments(get_engine_position(eng),
                             [thrust, 0, 0],
                             [0, 0, 0]
                             )

    return typeof(eng)(pfm, cj, Pm, ηp, tanks, get_engine_gyro_effects(eng))
end


get_thrust_setting(eng::C310EngineLeft, fcs::FCS) = get_value(fcs.t1)
get_thrust_setting(eng::C310EngineRight, fcs::FCS) = get_value(fcs.t2)


##----------------------------------------------------------------------------------------------------
## FCS

C310FCS() = C310FCS(# Cabin Inputs
                    RangeControl(0.0, [0, 1]),  # stick_longitudinal
                    RangeControl(0.0, [0, 1]),  # stick_lateral
                    RangeControl(0.0, [0, 1]),  # pedals
                    RangeControl(0.0, [0, 1]),  # thtl
                    # Controls
                    RangeControl(0.0, [-25.0, 35.0] .* DEG2RAD),  # elevator
                    RangeControl(0.0, [-18.0, 14.0] .* DEG2RAD),  # ailerons
                    RangeControl(0.0, [-27.0, 27.0] .* DEG2RAD),  # rudder
                    RangeControl(0.0, [0.0, 1.0]),                # t1
                    RangeControl(0.0, [0.0, 1.0])                 # t1
                    )

function set_stick_lon!(fcs::C310FCS, value, allow_out_of_range=false, throw_error=false)
    set_value!(fcs.stick_longitudinal, value)
    min, max = get_value_range(fcs.de)
    range = max - min
    set_value!(fcs.de, min + range * value, allow_out_of_range, throw_error)
end

function set_stick_lat!(fcs::C310FCS, value, allow_out_of_range=false, throw_error=false)
    set_value!(fcs.stick_lateral, value)
    min, max = get_value_range(fcs.da)
    range = max - min
    set_value!(fcs.da, min + range * value, allow_out_of_range, throw_error)
end

function set_pedals!(fcs::C310FCS, value, allow_out_of_range=false, throw_error=false)
    set_value!(fcs.pedals, value)
    min, max = get_value_range(fcs.dr)
    range = max - min
    set_value!(fcs.dr, min + range * value, allow_out_of_range, throw_error)
end

function set_thtl1!(fcs::C310FCS, value, allow_out_of_range=false, throw_error=false)
    set_value!(fcs.thtl, value)
    min, max = get_value_range(fcs.t1)
    range = max - min
    set_value!(fcs.t1, min + range * value, allow_out_of_range, throw_error)
end

function set_thtl2!(fcs::C310FCS, value, allow_out_of_range=false, throw_error=false)
    set_value!(fcs.thtl, value)
    min, max = get_value_range(fcs.t2)
    range = max - min
    set_value!(fcs.t2, min + range * value, allow_out_of_range, throw_error)
end

function set_thtl!(fcs::C310FCS, value, allow_out_of_range=false, throw_error=false)
    set_thtl1!(fcs, value, allow_out_of_range, throw_error)
    set_thtl2!(fcs, value, allow_out_of_range, throw_error)
end

function set_controls_trimmer!(fcs::C310FCS, slong, slat, ped, thtl,
    allow_out_of_range=true, throw_error=false)
    set_stick_lat!(fcs, slong, allow_out_of_range, throw_error)
    set_stick_lon!(fcs, slat, allow_out_of_range, throw_error)
    set_pedals!(fcs, ped, allow_out_of_range, throw_error)
    set_thtl!(fcs, thtl, allow_out_of_range, throw_error)
end

function get_controls_trimmer(fcs::C310FCS)
    [get_value(fcs.stick_longitudinal),
     get_value(fcs.stick_lateral),
     get_value(fcs.pedals),
     get_value(fcs.thtl)]
 end

function get_controls_ranges_trimmer(fcs::C310FCS)
    [get_value_range(fcs.stick_longitudinal),
     get_value_range(fcs.stick_lateral),
     get_value_range(fcs.pedals),
     get_value_range(fcs.thtl)]
end


##----------------------------------------------------------------------------------------------------
## Aircraft Model

function C310()
    pfm0 = PointForcesMoments(zeros(3), zeros(3), zeros(3))
    aero0 = C310Aerodynamics(pfm0, pfm0, pfm0, pfm0)
    engine_right = C310EngineRight()
    engine_left = C310EngineLeft()
    propulsion0 = Propulsion(
            PointForcesMoments(zeros(3), zeros(3), zeros(3)),
            0, 0, 0,
            [engine_right, engine_left],
            [0., 0., 0.]
            )

    # mass properties cannot be retrieved until ac is created... so:
    ac = C310(RigidSolid(0, zeros(3), zeros(3, 3)),
              pfm0,
              aero0,
              propulsion0)
    mass = get_fuel_mass_props(get_propulsion(ac)) +
           get_empty_mass_props(ac) +
           get_payload_mass_props(ac)
    C310(mass, pfm0, aero0, propulsion0)
end

# Name
function get_name(ac::C310)
    return "Cessna 310"
end

# GEOMETRIC PROPERTIES
get_wing_area(ac::C310) = 175.0 * FT2M^2
get_wing_span(ac::C310) = 36.5 * FT2M
get_chord(ac::C310) = 4.9 * FT2M
# Aerodynamic Reference Point
get_arp(ac::C310) = [46, 0, 8.6] .* IN2M

# MASS PROPERTIES
function get_empty_mass_props(ac::C310)
    RigidSolid(
        2950 * LB2KG,                            # Empty mass
        [46, 0, 8.6] .* IN2M,                    # Empty CG
        [8884      0      0;                     # Empty inertia
            0   1939      0;
            0      0  11001]  .* SLUGFT2_2_KGM2
    )
end

# Crew and cargo
get_pilot_mass_props(ac::C310) = PointMass(180 * LB2KG, [37, -14, 24] .* IN2M)
get_copilot_mass_props(ac::C310) = PointMass(180 * LB2KG, [37, 14, 24] .* IN2M)
get_lugage_mass_props(ac::C310) = PointMass(100 * LB2KG, [90, -0, 24] .* IN2M)

get_payload_mass_props(ac::C310) = (get_pilot_mass_props(ac) +
                                    get_copilot_mass_props(ac) +
                                    get_lugage_mass_props(ac)
                                    )

                                    
##----------------------------------------------------------------------------------------------------
