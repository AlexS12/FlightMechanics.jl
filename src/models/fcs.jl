# -------------------- FCS --------------------
abstract type FCS end

# Interface that must be implemented
function set_stick_lon! end
function set_stick_lat! end
function set_pedals! end
function set_thtl! end

# Interface to be trimmed
function set_controls_trimmer! end
function get_controls_trimmer end
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


function set_controls!(
    fcs::FCS, c::StickPedalsLeverControls; allow_out_of_range=false, throw_error=false
    )
    set_stick_lon!(fcs, c.stick_lon, allow_out_of_range, throw_error)
    set_stick_lat!(fcs, c.stick_lat, allow_out_of_range, throw_error)
    set_pedals!(fcs, c.pedals, allow_out_of_range, throw_error)
    set_thtl!(fcs, c.thrust_lever, allow_out_of_range, throw_error)
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
