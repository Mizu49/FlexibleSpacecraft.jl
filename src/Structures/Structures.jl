"""
    Structures

module for wrapping all the submodules for the structual dynamics for flexible spacecraft
"""
module Structures

using Reexport, YAML, StaticArrays

include("SpringMass.jl")
@reexport using .SpringMass

export FlexibleAppendage, SpacecraftAppendages, StructureSimData, initdatacontainer, update_structure, setstructure

"""
    StructureSimData

struct of the data container for the states and inputs of the structural response of the flexible appendages

# Fields

* `state::AbstractVector{<:AbstractVector}`: data container for the state vector trajectory of the flexible appendage
* `physicalstate::AbstractVector{<:AbstractVector}`: data container for the physical displacement and velocity trajectory of the flexible appendage
* `controlinput::AbstractVector{<:AbstractVector}`: data container for the control input trajectory
* `disturbance::AbstractVector{<:AbstractVector}`: data container for the disturbance input trajectory
"""
struct StructureSimData

    state::AbstractVector{<:AbstractVector}
    physicalstate::AbstractVector{<:AbstractVector}
    controlinput::AbstractVector{<:AbstractVector}
    disturbance::AbstractVector{<:AbstractVector}

end

"""
    FlexibleAppendage

struct that accomodates the configuration for a flexible appendage of the spacecraft
"""
struct FlexibleAppendage
    # name of the flexible appendage
    name::String
    # dynamics model that is employed for simulation of this appendage
    model::DataType

    # container for the simulation data
    simdata::StructureSimData

    FlexibleAppendage() = begin

        name = nothing
        model = nothing
        simdata = nothing

        new(name, model, simdata)
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
    initdatacontainer

initializer for the data container for structural simulation

# Arguments

* `model::StateSpace`: simulation model for the flexible appendage
* `initphysicalstate::Vector`: initial physical state value of the flexible appendage
* `datanum::Int`: numbers of the simulation data
"""
function initdatacontainer(model::StateSpace, initphysicalstate::Vector, datanum::Int)::StructureSimData

    # physical state vector (physical coordinate)
    physicalstate = [zeros(SVector{model.dimstate}) for _ in 1:datanum]
    physicalstate[1] = SVector{model.dimstate}(initphysicalstate)

    # state vector (modal coordinate)
    state = [zeros(SVector{model.dimstate}) for _ in 1:datanum]
    state[1] = physicalstate2modalstate(model, initphysicalstate)

    controlinput = [zeros(SVector{model.dimctrlinput}) for _ in 1:datanum]
    disturbance = [zeros(SVector{model.dimdistinput}) for _ in 1:datanum]

    return StructureSimData(state, physicalstate, controlinput, disturbance)
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
