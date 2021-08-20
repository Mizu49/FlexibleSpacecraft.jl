using Documenter

# Build documentation
makedocs(
    sitename="FlexibleSpacecraft.jl",
    pages = [
        "home" => "index.md",
        "Development" => [
            "Developer's Guideline" => "development/developer's-guideline.md",
            "Environment" => "development/environment.md"
        ],
        "Dynamics" => [
            "Frames" => "dynamics/frames.md",
        ]
    ]
)

# Deploy documentation to `gh-pages` branch
deploydocs(
    repo = "github.com/Mizu49/FlexibleSpacecraft.jl.git",
    devbranch = "dev-build",
    push_preview = true,
)
