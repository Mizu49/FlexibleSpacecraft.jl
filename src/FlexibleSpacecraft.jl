module FlexibleSpacecraft

using Reexport

@reexport using LinearAlgebra, GLMakie, StaticArrays, GeometryBasics

include("Core/UtilitiesBase.jl")
@reexport using .UtilitiesBase

include("DataContainers/DataContainers.jl")
@reexport using .DataContainers

include("DataContainers/Frames.jl")
@reexport using .Frames

# Include module `OrbitBase`
include("OrbitalDynamics/OrbitBase.jl")
@reexport using .OrbitBase

# Inculde module `AttitudeDisturbance`
include("AttitudeDynamics/AttitudeDisturbance.jl")
@reexport using .AttitudeDisturbance

include("AttitudeDynamics/DynamicsBase.jl")
@reexport using .DynamicsBase

include("AttitudeDynamics/KinematicsBase.jl")
@reexport using .KinematicsBase

include("AttitudeDynamics/Evaluation.jl")
@reexport using .Evaluation

# Include module `VisualizationBase`
include("Visualization/VisualizationBase.jl")
@reexport using .VisualizationBase

include("Structures/StructureDisturbance.jl")
@reexport using .StructureDisturbance

include("Structures/StructuresBase.jl")
@reexport using .StructuresBase

include("AttitudeControl/AttitudeControlBase.jl")
@reexport using .AttitudeControlBase

include("SimulationAPI/DataAPI.jl")
@reexport using .DataAPI

include("SimulationAPI/ParameterSettingBase.jl")
@reexport using .ParameterSettingBase

include("Core/SimulationCore.jl")
@reexport using .SimulationCore

include("CLI/CLI.jl")

end # module
