using Documenter
using FlightMechanics


makedocs()

deploydocs(
    deps = Deps.pip("mkdocs", "python-markdown-math"),
    repo = "github.com/AlexS12/FlightMechanics.jl.git",
    julia = "0.7"
)