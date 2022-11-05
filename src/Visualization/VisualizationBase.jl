"""
    module VisualizationBase

module of functions that show us the beautiful figures of spacecraft attitude dynamics
"""
module VisualizationBase

using Reexport

include("FramePlot.jl")
@reexport using .FramePlot

include("AttitudeVisualization.jl")
@reexport using .AttitudeVisualization

end
