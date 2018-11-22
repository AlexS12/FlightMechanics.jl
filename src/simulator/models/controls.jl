export Control, RangeControl, DiscreteControl,
    get_value, get_value_range, get_value_choices,
    set_value


# --------- Control ---------
abstract type Control end

get_value(c::Control) = c.value


# --------- RangeControl ---------
mutable struct RangeControl<:Control
    value::Number
    value_range::Array{Number, 1}
end

get_value_range(c::RangeControl) = c.value_range

# function set_value(c::RangeControl, val)
#     min, max = get_value_range(c)
#     if min <= val <= max
#         c.value = val
#     elseif val < min
#         c.value = min
#     else
#         c.value = max
#         #throw(DomainError(val, "val must be between min and max"))
#     end
# end

# TODO: for trimmer to work, values out of the domain are allowed.
# an optimization method with boundaries must be sought
function set_value(c::RangeControl, val)
    c.value = val
        #throw(DomainError(val, "val must be between min and max"))
end


# --------- DiscreteControl ---------
mutable struct DiscreteControl<:Control
    value::Int
    value_choices::Array{Int, 1}
end

get_value_choices(c::DiscreteControl) = c.value_choices

function set_value(c::DiscreteControl, val)
    if val in get_value_choices(c)
        c.value = val
    else
        throw(DomainError(val, "val not in dicrete control options"))
    end
end
