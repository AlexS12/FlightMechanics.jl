using FlightMechanics.Models
using FlightMechanics
using FlightMechanics.Aircrafts


export F16FCS,
    set_stick_lon, set_stick_lat, set_pedals,
    set_thtl,
    set_controls_trimmer, get_controls_trimmer,
    get_controls_ranges_trimmer


# TODO: implement also rate limits and time constants
struct F16FCS<:FCS
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
    cpl::RangeControl
end

# TODO: Move to FCS in models
get_thrust(fcs::F16FCS) = get_value(fcs.cpl)

# TODO: move this constants to aircraft parameters
const DE_MAX = 25.0  # deg
const DA_MAX = 20.0  # deg  #XXX: In Stevens' book says 21.5 deg (Appendix A Section A.4)
const DR_MAX = 30.0  # deg

F16FCS() = F16FCS(# Cabin Inputs
                    RangeControl(0.0, [0, 1]),  # stick_longitudinal
                    RangeControl(0.0, [0, 1]),  # stick_lateral
                    RangeControl(0.0, [0, 1]),  # pedals
                    RangeControl(0.0, [0, 1]),  # thtl
                    # Controls
                    RangeControl(0.0, [-DE_MAX, DE_MAX] .* DEG2RAD),  # elevator
                    RangeControl(0.0, [-DA_MAX, DA_MAX] .* DEG2RAD),  # ailerons
                    RangeControl(0.0, [-DR_MAX, DR_MAX] .* DEG2RAD),  # rudder
                    # Commanded power level
                    RangeControl(0.0, [0.0, 100.]),                # CPL
                    )

function set_stick_lon(fcs::F16FCS, value, allow_out_of_range=false, throw_error=false)
    set_value(fcs.stick_longitudinal, value)
    min, max = get_value_range(fcs.de)
    range = max - min
    set_value(fcs.de, min + range * value, allow_out_of_range, throw_error)
end

function set_stick_lat(fcs::F16FCS, value, allow_out_of_range=false, throw_error=false)
    set_value(fcs.stick_lateral, value)
    min, max = get_value_range(fcs.da)
    range = max - min
    set_value(fcs.da, min + range * value, allow_out_of_range, throw_error)
end

function set_pedals(fcs::F16FCS, value, allow_out_of_range=false, throw_error=false)
    set_value(fcs.pedals, value)
    min, max = get_value_range(fcs.dr)
    range = max - min
    set_value(fcs.dr, min + range * value, allow_out_of_range, throw_error)
end

function set_thtl(fcs::F16FCS, value, allow_out_of_range=false, throw_error=false)
    set_value(fcs.thtl, value)
    min, max = get_value_range(fcs.thtl)
    range = max - min
    set_value(fcs.cpl, Aircrafts.tgear(value), allow_out_of_range, throw_error)
end

function set_controls_trimmer(fcs::F16FCS, slong, slat, ped, thtl,
    allow_out_of_range=true, throw_error=false)
    set_stick_lat(fcs, slong, allow_out_of_range, throw_error)
    set_stick_lon(fcs, slat, allow_out_of_range, throw_error)
    set_pedals(fcs, ped, allow_out_of_range, throw_error)
    set_thtl(fcs, thtl, allow_out_of_range, throw_error)
end

function get_controls_trimmer(fcs::F16FCS)
    [get_value(fcs.stick_longitudinal),
     get_value(fcs.stick_lateral),
     get_value(fcs.pedals),
     get_value(fcs.thtl)]
 end

function get_controls_ranges_trimmer(fcs::F16FCS)
    [get_value_range(fcs.stick_longitudinal),
     get_value_range(fcs.stick_lateral),
     get_value_range(fcs.pedals),
     get_value_range(fcs.thtl)]
end
