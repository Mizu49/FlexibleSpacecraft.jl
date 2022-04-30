# This script allow you to build and preview documentation locally. Use this to check your contribution on documentation!

using LiveServer

println("Build and preview documentation locally")

# Build documentation
println("Building the documentation...")
include("make.jl")

println("Serving documentation locally...")

serve(dir = "docs/build/")
