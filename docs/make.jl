using Documenter
using LiveServer

println("Building documentation!")

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


println("Serve documentation locally!")
serve(dir = "build/")
