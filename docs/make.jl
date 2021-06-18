using Documenter
using LiveServer

println("Building documentation!")

makedocs(
    sitename="FlexibleSpacecraft.jl",
    pages = [
        "home" => "index.md",
        "Development" => [
            "Developer's Guideline" => "development/developer's-guideline.md",
        ]
    ]
)


println("Serve documentation locally!")
serve(dir = "build/")
