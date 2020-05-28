"""
    ResultsStore(t, ac, env, state, aerostate)

Store results for different timesteps.
"""
struct ResultsStore
    t :: Array{T, 1} where T <: Number
    ac :: Array{T, 1} where T <: Aircraft
    env :: Array{T, 1} where T <: Environment
    state :: Array{T, 1} where T <: State
    aerostate :: Array{T, 1} where T <: AeroState
end


function ResultsStore(t::Number, ac::Aircraft, env::Environment, state::State, aerostate::AeroState)
    ResultsStore([t], [ac], [env], [state], [aerostate])
end


function push!(results::ResultsStore, t, ac, env, state, aerostate)
    push!(results.t, t)
    push!(results.ac, ac)
    push!(results.env, env)
    push!(results.state, state)
    push!(results.aerostate, aerostate)
end


# Indexable collection
function getindex(results::ResultsStore, ii)
    return ResultsStore(results.t[ii], results.ac[ii], results.env[ii], results.state[ii], results.aerostate[ii])
end


function set_index!(results::ResultsStore, value, ii)
    results.t[ii] = value[1]
    results.ac[ii] = value[2]
    results.env[ii] = value[3]
    results.state[ii] = value[4]
    results.aerostate[ii] = value[5]
end
