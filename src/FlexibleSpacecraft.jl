module FlexibleSpacecraft

using Reexport

@reexport using LinearAlgebra


# Include module `AttitudeDynamics`
include("AttitudeDynamics.jl")
@reexport using .AttitudeDynamics


# Include module `TimeLine`
include("TimeLine.jl")
@reexport using .TimeLine

# Include module `PlotGenerator`
include("PlotGenerator.jl")
@reexport using .PlotGenerator as plt

end # module 