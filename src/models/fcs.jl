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
