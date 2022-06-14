module FlexibleSpacecraft

using Reexport

@reexport using LinearAlgebra, Plots, StaticArrays

include("Core/Utilities.jl")
@reexport using.Utilities

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

include("Structures/StructureDisturbance.jl")
@reexport using .StructureDisturbance

include("Structures/StructuresBase.jl")
@reexport using .StructuresBase

include("SimulationAPI/DataAPI.jl")
@reexport using .DataAPI

include("SimulationAPI/ParameterSettingBase.jl")
@reexport using .ParameterSettingBase

include("Core/SimulationCore.jl")
@reexport using .SimulationCore

include("CLI/CLI.jl")

end # module
