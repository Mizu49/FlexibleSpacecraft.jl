using Documenter

using FlexibleSpacecraft

# Build documentation
makedocs(
    sitename="FlexibleSpacecraft.jl",
    pages = [
        "home" => "index.md",
        "Example" => "examples/example.md",
        "Simulation configuration" => "simulation_config/simulation_config.md",
        "CLI" => "docs-CLI/index.md",
        "Dynamics" => [
            "Frames" => "dynamics/frames.md",
            "Attitude dynamics" => "dynamics/attitude.md",
            "Rigid body" => "dynamics/rigid-body.md"
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
    devbranch = "dev-build",
    push_preview = true,
)
