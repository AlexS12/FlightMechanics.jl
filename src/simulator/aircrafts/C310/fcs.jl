using FlightMechanics.Simulator.Models


export C310FCS,
    set_stick_lon, set_stick_lat, set_pedals,
    set_thrust1, set_thrust2, set_thrust,
    set_controls_trimmer, get_controls_trimmer,
    get_controls_ranges_trimmer


struct C310FCS<:FCS
    # Cabin controls
    stick_longitudinal::RangeControl
    stick_lateral::RangeControl
    pedals::RangeControl
    thrust1::RangeControl
    thrust2::RangeControl

    # Aerodynamic surfaces
    de::RangeControl
    da::RangeControl
    dr::RangeControl
    # Engine
    t1::RangeControl
    t2::RangeControl
end

C310FCS() = C310FCS(# Cabin Inputs
                    RangeControl(0.0, [0, 1]),  # stick_longitudinal
                    RangeControl(0.0, [0, 1]),  # stick_lateral
                    RangeControl(0.0, [0, 1]),  # pedals
                    RangeControl(0.0, [0, 1]),  # thrust1
                    RangeControl(0.0, [0, 1]),  # thrust2
                    # Controls
                    RangeControl(0.0, [-25.0, 35.0] .* DEG2RAD),  # elevator
                    RangeControl(0.0, [-18.0, 14.0] .* DEG2RAD),  # ailerons
                    RangeControl(0.0, [-27.0, 27.0] .* DEG2RAD),  # rudder
                    RangeControl(0.0, [0.0, 1.0]),                # t1
                    RangeControl(0.0, [0.0, 1.0])                 # t1
                    )

function set_stick_lon(fcs::C310FCS, value, allow_out_of_range=false, throw_error=false)
    set_value(fcs.stick_longitudinal, value)
    min, max = get_value_range(fcs.de)
    range = max - min
    set_value(fcs.de, min + range * value, allow_out_of_range, throw_error)
end

function set_stick_lat(fcs::C310FCS, value, allow_out_of_range=false, throw_error=false)
    set_value(fcs.stick_lateral, value)
    min, max = get_value_range(fcs.da)
    range = max - min
    set_value(fcs.da, min + range * value, allow_out_of_range, throw_error)
end

function set_pedals(fcs::C310FCS, value, allow_out_of_range=false, throw_error=false)
    set_value(fcs.pedals, value)
    min, max = get_value_range(fcs.dr)
    range = max - min
    set_value(fcs.dr, min + range * value, allow_out_of_range, throw_error)
end

function set_thrust1(fcs::C310FCS, value, allow_out_of_range=false, throw_error=false)
    set_value(fcs.thrust1, value)
    min, max = get_value_range(fcs.t1)
    range = max - min
    set_value(fcs.t1, min + range * value, allow_out_of_range, throw_error)
end

function set_thrust2(fcs::C310FCS, value, allow_out_of_range=false, throw_error=false)
    set_value(fcs.thrust2, value)
    min, max = get_value_range(fcs.t2)
    range = max - min
    set_value(fcs.t2, min + range * value, allow_out_of_range, throw_error)
end

function set_thrust(fcs::C310FCS, value, allow_out_of_range=false, throw_error=false)
    set_thrust1(fcs, value, allow_out_of_range, throw_error)
    set_thrust2(fcs, value, allow_out_of_range, throw_error)
end

function set_controls_trimmer(fcs::C310FCS, slong, slat, ped, thrust,
    allow_out_of_range=true, throw_error=false)
    set_stick_lat(fcs, slong, allow_out_of_range, throw_error)
    set_stick_lon(fcs, slat, allow_out_of_range, throw_error)
    set_pedals(fcs, ped, allow_out_of_range, throw_error)
    set_thrust(fcs, thrust, allow_out_of_range, throw_error)
end

function get_controls_trimmer(fcs::C310FCS)
    [get_value(fcs.stick_longitudinal),
     get_value(fcs.stick_lateral),
     get_value(fcs.pedals),
     get_value(fcs.thrust1)]
 end

function get_controls_ranges_trimmer(fcs::C310FCS)
    [get_value_range(fcs.stick_longitudinal),
     get_value_range(fcs.stick_lateral),
     get_value_range(fcs.pedals),
     get_value_range(fcs.thrust1)]
end
