export Engine, Propulsion


abstract type Engine end


struct Propulsion
    pfm::PointForcesMoments
    cj::Number
    power::Number
    efficiency::Number
    engines::Array{Engine, 1}
end
