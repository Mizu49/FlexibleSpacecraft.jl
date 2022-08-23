using Documenter

using FlexibleSpacecraft

# Build documentation
makedocs(
    sitename="FlexibleSpacecraft.jl",
    pages = [
        "home" => "index.md",
        "Example" => "examples/example.md",
        "Simulation" => [
            "Core" => "simulation/simulation_cores/example.md",
        "Simulation" => [
            "Core" => "simulation/simulation_core.md",
            "Configuration" => "simulation/simulation_config.md"
        ],
            "Configuration" => "simulation/simulation_config.md"
        ],
        "CLI" => "docs-CLI/index.md",
        "Attitude" => [
            "Kinematics" => "dynamics/attitude.md",
            "Dynamics" => "dynamics/dynamics.md",
            "Frames" => "dynamics/frames.md",
            "Rigid body" => "dynamics/rigid-body.md",
            "Orbit" => "dynamics/orbit.md",
            "Disturbance input" => "dynamics/disturbance.md",
            "Structures" => [
                "Index" => "dynamics/structures/structures.md",
                "Spring mass model" => "dynamics/structures/SpringMass.md"
            ]
        ],
        "Libraries" => [
            "API" => "docs-module/docs-SimulationAPI.md",
            "Data containers" => "docs-module/docs-DataContainers.md",
            "Plot recipe" => "docs-module/docs-PlotRecipe.md"
        ],
        "Development" => [
            "Developer's Guideline" => "development/developer's-guideline.md",
            "Environment" => "development/environment.md"
        ]
    ]
)

# Deploy documentation to `gh-pages` branch
deploydocs(
    repo = "github.com/Mizu49/FlexibleSpacecraft.jl.git",
    devbranch = "develop",
    push_preview = true,
)
