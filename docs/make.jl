using Documenter

using FlexibleSpacecraft

# Build documentation
makedocs(
    sitename="FlexibleSpacecraft.jl",
    pages = [
        "home" => "index.md",
        "Example" => "example.md",
        "Development" => [
            "Developer's Guideline" => "development/developer's-guideline.md",
            "Environment" => "development/environment.md"
        ],
        "Dynamics" => [
            "Frames" => "dynamics/frames.md",
        ],
        "Libraries" => [
            "API" => "docs-module/docs-SimulationAPI.md",
            "Data containers" => "docs-module/docs-DataContainers.md",
            "Plot recipe" => "docs-module/docs-PlotRecipe.md"
        ]
    ]
)

# Deploy documentation to `gh-pages` branch
deploydocs(
    repo = "github.com/Mizu49/FlexibleSpacecraft.jl.git",
    devbranch = "dev-build",
    push_preview = true,
)
