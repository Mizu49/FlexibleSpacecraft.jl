module FlexibleSpacecraft

using Reexport

@reexport using LinearAlgebra


# Include module `RigidBody.jl`
include("RigidBody.jl")
@reexport using .RigidBody


# Include module `TimeLine`
include("TimeLine.jl")
@reexport using .TimeLine

# Include module `PlotRecipe`
include("PlotRecipe.jl")
@reexport using .PlotRecipe

# Include module `Orbit`
include("Orbit.jl")
@reexport using .Orbit

# Inculde module `Disturbance`
include("Disturbance.jl")
@reexport using .Disturbance

end # module
