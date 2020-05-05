# -------------------- FCS --------------------
abstract type FCS end

# Interface that must be implemented
function set_stick_lon! end
function set_stick_lat! end
function set_pedals! end
function set_thtl! end

# Interface to be trimmed
function get_controls_ranges_trimmer end


function copy(fcs::FCS)
    fcs_type = typeof(fcs)
    fcs_ = fcs_type(
        [copy(getfield(fcs, name)) for name in fieldnames(fcs_type)]...
        )
    return fcs_
end

# -------------------- Controls --------------------
"Controls are the input to a FCS."
abstract type Controls end

# Interface that must be implemented
"""
    set_controls!(fcs, controls; allow_out_of_range=false, throw_error=false)

Sets a FCS with the given controls.
If allow_out_of_range values outside the range will be allowed. This is useful in some
situations such as the trimming process. Otherwise, the values will be set to the limit
if throw_error is false (otherwise an error will be thrown)
"""
function set_controls! end

function get_controls_array end
function get_stick_lon end
function get_stick_lat end
function get_pedals end
function get_thrust_lever end

get_n_controls(c::Controls) = size(get_controls_array)


"""
    StickPedalsLeverControls(stick_lon, stick_lat, pedals, thrust_lever)

Classical controls stick longitudinal and lateral setting, pedals, and thrust lever.
"""
struct StickPedalsLeverControls <: Controls
    stick_lon :: Number
    stick_lat :: Number
    pedals :: Number
    thrust_lever :: Number
end


get_controls_array(c::StickPedalsLeverControls) = [
    get_stick_lon(c), get_stick_lat(c), get_pedals(c), get_thrust_lever(c)
    ]
get_stick_lon(c::StickPedalsLeverControls) = c.stick_lon
get_stick_lat(c::StickPedalsLeverControls) = c.stick_lat
get_pedals(c::StickPedalsLeverControls) = c.pedals
get_thrust_lever(c::StickPedalsLeverControls) = c.thrust_lever


function set_controls!(
    fcs::FCS, c::StickPedalsLeverControls; allow_out_of_range=false, throw_error=false
    )
    set_stick_lon!(fcs, get_stick_lon(c), allow_out_of_range, throw_error)
    set_stick_lat!(fcs, get_stick_lat(c), allow_out_of_range, throw_error)
    set_pedals!(fcs, get_pedals(c), allow_out_of_range, throw_error)
    set_thtl!(fcs, get_thrust_lever(c), allow_out_of_range, throw_error)
end


# -------------------- ControlsStream --------------------
"""
Controls are defined normally in a strem (function or callable) that returns the control
value a certain time.
"""
abstract type ControlsStream end

# Interface that must be implemented
"""
    get_controls(cs::ControlsStream, t)

Get the control values of a control stream at a certain instant t. Return Controls.
"""
function get_controls end


"""
    StickPedalsLeverStream(stick_lon, stick_lat, pedals, thrust_lever)

Classical controls stick longitudinal and lateral setting, pedals, and thrust lever.
Inputs are of type Input or Array{T, 1} where T<:Input
"""
struct StickPedalsLeverStream <: ControlsStream
    stick_lon_stream :: Union{Input, Array{T, 1}} where T<:Input
    stick_lat_stream :: Union{Input, Array{T, 1}} where T<:Input
    pedals_stream :: Union{Input, Array{T, 1}} where T<:Input
    thrust_lever_stream :: Union{Input, Array{T, 1}} where T<:Input
end


function get_controls(cs::StickPedalsLeverStream, t)
    slon = get_value(cs.stick_lon_stream, t)
    slat = get_value(cs.stick_lat_stream, t)
    ped = get_value(cs.pedals_stream, t)
    lev = get_value(cs.thrust_lever_stream, t)
    return StickPedalsLeverControls(slon, slat, ped, lev)
end
