using Documenter
using FlightMechanics


makedocs(authors="Alejandro Saez Mollejo",
         pages = [
         "Home" => "index.md",
         "API Reference" => ["api-reference.md"],
         "Basics" => [
           "isa.md",
           "coordinate-systems.md",
           "dynamic_models.md",
           "anemometry.md",
           "kinematics.md"
         ],
         ])

deploydocs(
    deps = Deps.pip("mkdocs", "python-markdown-math"),
    repo = "github.com/AlexS12/FlightMechanics.jl.git",
    julia = "0.7"
)
