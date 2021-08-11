module FlexibleSpacecraft

using Reexport

@reexport using LinearAlgebra


# Include module `RigidBodyAttitudeDynamics.jl`
include("RigidBodyAttitudeDynamics.jl")
@reexport using .RigidBodyAttitudeDynamics


# Include module `TimeLine`
include("TimeLine.jl")
@reexport using .TimeLine

# Include module `PlotGenerator`
include("PlotGenerator.jl")
@reexport using .PlotGenerator

# Include module `Orbit`
include("Orbit.jl")
@reexport using .Orbit

end # module
