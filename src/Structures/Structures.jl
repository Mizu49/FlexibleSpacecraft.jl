"""
    Structures

module for wrapping all the submodules for the structual dynamics for flexible spacecraft
"""
module Structures

using Reexport

include("SpringMass.jl")
@reexport using .SpringMass

end
