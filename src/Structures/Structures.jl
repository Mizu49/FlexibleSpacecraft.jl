"""
    Structures

module for wrapping all the submodules for the structual dynamics for flexible spacecraft
"""
module Structures

using Reexport, YAML, StaticArrays

include("SpringMass.jl")
@reexport using .SpringMass

export FlexibleAppendage, SpacecraftAppendages, update_structure, setstructure

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
    SpacecraftAppendages

struct that accomodates the configurations for the multiple flexible appendages
"""
struct SpacecraftAppendages

    appendages::SVector

    SpacecraftAppendages(configs::AbstractVector) = begin
        # numbers of appendages to be simulated
        num_appendages = size(configs, 1)

        appendages = SVector{num_appendages, Any}(undef for _ in 1:num_appendages)

        new(appendages)
    end
end

"""
    setstructure

API function to define the model to
"""
function setstructure(configfilepath::String)

    lawread = YAML.load_file(configfilepath)

    if haskey(lawread, "modeling") == false
        throw(ErrorException("`model` is undefined in configuration file `$configfilepath`"))
    end

    if lawread["modeling"] == "spring-mass"
        (structureparams, structuresimmodel) = defmodel(lawread)
    else
        throw(ErrorException("no matching modeling method for \"$(lawread["modeling"])\""))
    end

    return (structureparams, structuresimmodel)
end

"""
    update_structure

function to calculate the time evolution of the structural motion of the flexible appendages
"""
function update_structure(config, angularvelocity::SVector{3, <:Real}, controlinput::AbstractVector)

    return nothing
end

end
