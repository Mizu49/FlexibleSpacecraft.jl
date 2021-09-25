module FlexibleSpacecraft

using Reexport

@reexport using LinearAlgebra

# Include module `TimeLine`
include("TimeLine.jl")
@reexport using .TimeLine

# Include module `Orbit`
include("Orbit.jl")
@reexport using .Orbit

# Inculde module `Disturbance`
include("Disturbance.jl")
@reexport using .Disturbance

# Include module `RigidBody.jl`
include("RigidBody.jl")
@reexport using .RigidBody

# Include module `PlotRecipe`
include("PlotRecipe.jl")
@reexport using .PlotRecipe

include("Frame.jl")
@reexport using .Frame

include("runsimulation.jl")
export runsimulation

end # module
