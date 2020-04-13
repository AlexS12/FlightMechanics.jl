abstract type FCS end

function copy(fcs::FCS)
    fcs_type = typeof(fcs)
    fcs_ = fcs_type(
        [copy(getfield(fcs, name)) for name in fieldnames(fcs_type)]...
        )
    return fcs_
end
