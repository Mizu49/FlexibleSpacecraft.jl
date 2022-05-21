module FlexibleSpacecraft

using Reexport

@reexport using LinearAlgebra, Plots, StaticArrays

include("DataContainers/DataContainers.jl")
@reexport using .DataContainers

include("DataContainers/Frames.jl")
@reexport using .Frames

# Include module `Orbit`
include("OrbitalDynamics/Orbit.jl")
@reexport using .Orbit

# Inculde module `Disturbance`
include("Disturbances/Disturbance.jl")
@reexport using .Disturbance

include("AttitudeDynamics/DynamicsBase.jl")
@reexport using .DynamicsBase

include("AttitudeDynamics/Attitude.jl")
@reexport using .Attitude

include("AttitudeDynamics/Evaluation.jl")
@reexport using .Evaluation

# Include module `PlotRecipe`
include("PlotRecipes/PlotRecipe.jl")
@reexport using .PlotRecipe

include("SimulationAPI/ParameterSetting.jl")
@reexport using .ParameterSetting

include("Structures/Structures.jl")
@reexport using .Structures

include("SimulationAPI/DataAPI.jl")
@reexport using .DataAPI

include("SimulationAPI/runsimulation.jl")
export runsimulation

include("CLI/CLI.jl")

end # module
