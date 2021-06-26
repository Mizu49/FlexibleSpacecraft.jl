using Documenter

# Build documentation
makedocs(
    sitename="FlexibleSpacecraft.jl",
    pages = [
        "home" => "index.md",
        "Development" => [
            "Developer's Guideline" => "development/developer's-guideline.md",
            "Environment" => "development/environment.md"
        ]
    ]
)

# Deploy documentation to `gh-pages` branch
deploydocs(
    repo = "github.com/Mizu49/FlexibleSpacecraft.jl.git",
)
