using FlightMechanics
using FlightMechanics.Models

import FlightMechanics.Models:
    get_pfm, get_cj, get_power, get_efficiency, get_tanks,
    get_engine_position, get_engine_orientation, get_engine_gyro_effects,
    calculate_engine

export F16Engine,
    get_pfm, get_cj, get_power, get_efficiency, get_tanks,
    get_engine_position, get_engine_orientation,
    calculate_engine


struct F16Engine<:Engine
    pfm::PointForcesMoments
    cj::Number
    power::Number
    efficiency::Number
    tanks::Array{RigidSolid, 1}
    h::Array{T, 1} where T<:Number  # angular momentum [kg·m²/s]
end

function F16Engine()
    F16Engine(
        PointForcesMoments(zeros(3), zeros(3), zeros(3)),
        0,
        0,
        0,
        # Fuel tanks can be obtained from JSBSim model
        [PointMass(0 * LB2KG, [0., 0., 0.] .* IN2M)],
        [160.0*SLUGFT2_2_KGM2, 0.0, 0.0]
        )

end

# TODO: Engine position and orientation
get_engine_position(prop::F16Engine) = [0.35 * 11.32 * FT2M, 0.0, 0.0]
get_engine_orientation(prop::F16Engine) = [0., 0., 0.] .* DEG2RAD
get_engine_gyro_effects(prop::F16Engine) = [160.0*SLUGFT2_2_KGM2, 0.0, 0.0]  # [kg·m²/s]


# ENGINE DATA
"""
    tgear(thtl)

Given the thottle setting (0 ≤ thtl ≤ 1) calculate the commanded power level
(0 ≤ cpow ≤ 100)
"""
function tgear(thtl)
    if thtl <= .77
     cpow = 64.94 * thtl
    else
     cpow = 217.38 * thtl - 117.38
    end
    return cpow
end

"""
    pdot(p3, p1)

Rate of change of power with time using a first order lag (%/s)

# Arguments
p3::Number  Actual power (%)
p1::Number  Power command (%)
"""
function pdot(p3, p1)
    if p1 >= 50.
        if p3 >= 50.
            t = 5.
            p2 = p1
        else
            p2 = 60.
            t = rtau(p2-p3)
        end
    else
        if p3 >= 50.
            t = 5.
            p2 = 40.
        else
            p2 = p1
            t = rtau(p2-p3)
        end
    end
    pd = t * (p2 - p3)
    return pd
end

"""
    rtau(dp)

Calculate the reciprocal time constant (1/s) for a first-order thrust lag given
the difference between the commanded power level (%) and the actual power
level (%).
"""
function rtau(dp)
    if dp <= 25.
        rt = 1.0  # Reciprocal time constant
    elseif dp >= 50.
        rt = 0.1
    else
        rt = 1.9 - 0.036 * dp
    end
    return rt
end


idle_data =
    [ 1060.  670.   880.   1140.  1500.  1860.
      635.   425.   690.   1010.  1330.  1700.
      60.    25.    345.   755.   1130.  1525.
     -1020. -710.  -300.   350.   910.   1360.
     -2700. -1900. -1300. -247.   600.   1100.
     -3600. -1400. -595.  -342.  -200.   700.]'

# Mil data
mil_data =
    [12680. 9150.  6200.  3950.  2450. 1400.
     12680. 9150.  6313.  4040.  2470. 1400.
     12610. 9312.  6610.  4290.  2600. 1560.
     12640. 9839.  7090.  4660.  2840. 1660.
     12390. 10176. 7750.  5320.  3250. 1930.
     11680. 9848.  8050.  6100.  3800. 2310.]'

# Max data
max_data =
    [20000. 15000. 10800. 7000.  4000. 2500.
     21420. 15700. 11225. 7323.  4435. 2600.
     22700. 16860. 12250. 8154.  5000. 2835.
     24240. 18910. 13760. 9285.  5700. 3215.
     26070. 21075. 15975. 11115. 6860. 3950.
     28886. 23319. 18300. 13484. 8642. 5057.]'


"""
    thrust(pow, alt, rmach)

Given the commanded power level (%), the altitude (ft) and the Mach number,
calculate the engine thrust (lbf)
"""
function calculate_thrust(pow, alt, rmach)

    h = 0.0001 * alt
    i = floor(Int, h)
    if i >= 5
        i=4
    end

    dh = h - float(i)

    rm = 5.0 * rmach
    m = floor(Int, rm);
    if m >= 5
        m = 4
    end

    dm = rm - float(m)
    cdh = 1.0 - float(dh)

    i=i+1;
    m=m+1;

    s = mil_data[i, m] * cdh + mil_data[i+1, m] * dh
    t = mil_data[i, m+1] *cdh + mil_data[i+1, m+1] * dh
    tmil = s + (t - s) * dm

    if pow < 50.
        s = idle_data[i, m] * cdh + idle_data[i+1, m] * dh
        t = idle_data[i,m+1] * cdh + idle_data[i+1, m+1] * dh
        tidl = s + (t - s) * dm
        thrst = tidl + (tmil - tidl) * pow * 0.02
    else
        s = max_data[i, m] * cdh + max_data[i+1, m] * dh
        t = max_data[i, m+1] * cdh + max_data[i+1, m+1] * dh
        tmax = s + (t-s) * dm
        thrst = tmil + (tmax - tmil) * (pow-50.) * 0.02
    end

    return thrst
    end


"""
The F-16 engine power response is modelled by a first-order lag (pdot function).
The rest of the model consists of thottle gearing (tgear) and lookup tables for
thrust as a function of operating power level, altitude and Mach.

Lookup tables have rows corresponding to Mach from 0 to 1 in increments of 0.2
and columns correspond to altitudes from 0 to 50000ft in increments of 10000ft.
There is a table for each power level: idle, military and maximum. Results can
be extrapolated beyond the boundaries but results may be unrealistic.

# References

Reimplemented from:

- [1] Stevens, B. L., Lewis, F. L., & Johnson, E. N. (2015). Aircraft control
 and simulation: dynamics, controls design, and autonomous systems. John Wiley
 & Sons. (page 715)
"""
function calculate_engine(eng::F16Engine, fcs::FCS, aerostate::AeroState,
                          state::State; consume_fuel=false)

    get_thrust(fcs) = get_value(fcs.cpl)        # temporarily here # TODo: move to model/fcs.jl
    pow = get_thrust(fcs)  # Commanded power: between 0 and 100
    Mach = get_mach(aerostate)
    # TODO: Should be altitude
    height = get_height(state) * M2FT

    thrust = calculate_thrust(pow, height, Mach)
    thrust *= LBF2N
    cj = 0.0
    Pm = thrust * get_tas(aerostate)
    ηp = NaN

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
