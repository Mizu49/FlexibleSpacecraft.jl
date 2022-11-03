"""
    module VisualizationBase

module of functions that show us the beautiful figures of spacecraft attitude dynamics
"""
module VisualizationBase

using Reexport

include("FramePlot.jl")
@reexport using .FramePlot

include("PhysicalQuantity.jl")
@reexport using .PhysicalQuantity

end
