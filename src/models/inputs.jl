abstract type Input end


"""
    get_value(input::Constant, time)

Get input value at time t.
"""
function get_value end


"""
    ConstantInput(val)

Constant value input.
"""
struct ConstantInput <: Input
    val::Float64
end


"""
    StepInput(val, tini, tfin)

Step input. Takes value from tini to tfin and 0 otherwise.
"""
struct StepInput <: Input
    val::Float64
    tini::Float64
    tfin::Float64
end


"""
    DoubletInput(val, tini, tfin)

Symmetric doublet input with val peak to peak amplitude. First positive.
"""
DoubletInput(val, tini, tfin) = vcat(
    StepInput(0.5*val, tini, 0.5*(tini+tfin)),
    StepInput(-0.5*val, 0.5*(tini+tfin), tfin)
)


"""
    InverseDoubletInput(val, tini, tfin)

Symmetric doublet input with val peak to peak amplitude. First negative.
"""
InverseDoubletInput(val, tini, tfin) = vcat(
    StepInput(-0.5*val, tini, 0.5*(tini+tfin)),
    StepInput(0.5*val, 0.5*(tini+tfin), tfin)
)


"""
    RampInput(val, tini, tend)

Ramp input. Takes value at tend increasing linearly from tini to tend.
"""
struct RampInput <: Input
    val::Float64
    tini::Float64
    tfin::Float64
end


"""
    SinusoidalInput(A, f, ϕ, tini, tfin)

Sinusoidal input. Takes a A/2 * sin(2πf (t-tini) + ϕ) value between tini and tfin.
Value is 0 otherwise.
"""
struct SinusoidalInput <: Input
    "Peak to peak amplitude"
    A::Float64
    "Frequency (1/s)"
    f::Float64
    "Phase (rad)"
    ϕ::Float64
    tini::Float64
    tfin::Float64
end


# --------- get_value methods ---------
get_value(input::ConstantInput, time) = input.val
get_value(input::StepInput, time) = input.tini <= time < input.tfin ? input.val : 0.

function get_value(input::RampInput, time)
    if input.tini <= time < input.tfin
        return input.val * (time - input.tini) / (input.tfin - input.tini)
    else
        return 0.
    end
end


function get_value(input::SinusoidalInput, time)
    if input.tini <= time < input.tfin
        return 0.5 * input.A * sin(2π * input.f * (time - input.tini) + input.ϕ)
    else
        return 0.
    end
end


function get_value(input_arr::Array{T,1}, t) where T<:Input
    value = 0.0
    for input in input_arr
        value = value + get_value(input, t)
    end
    return value
end
