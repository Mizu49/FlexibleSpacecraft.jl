"""
    Structures

module for wrapping all the submodules for the structual dynamics for flexible spacecraft
"""
module Structures

using Reexport, StaticArrays

include("SpringMass.jl")
@reexport using .SpringMass

export FlexibleAppendage, AllAppendages, update_structure

"""
    FlexibleAppendage

struct that accomodates the configuration for a flexible appendage of the spacecraft
"""
struct FlexibleAppendage
    # name of the flexible appendage
    name::String
    # dynamics model that is employed for simulation of this appendage
    model::DataType

    FlexibleAppendage() = begin

        new(name, model)
    end
end

"""
    AllAppendages

struct that accomodates the configurations for the multiple flexible appendages
"""
struct AllAppendages

    appendages::SVector

    AllAppendages(configs::AbstractVector) = begin
        # numbers of appendages to be simulated
        num_appendages = size(configs, 1)

        appendages = SVector{num_appendages, Any}(undef for _ in 1:num_appendages)

        new(appendages)
    end
end

"""
    update_structure

function to calculate the time evolution of the structural motion of the flexible appendages
"""
function update_structure(config, angularvelocity::SVector{3, <:Real}, controlinput::AbstractVector)



end

end
