"""
    StructuresBase

module for wrapping all the submodules for the structual dynamics for flexible spacecraft
"""
module StructuresBase

using Reexport, YAML, StaticArrays
using ..Utilities, ..StructureDisturbance

include("SpringMass.jl")
@reexport using .SpringMass

include("NoAppendages.jl")
@reexport using .NoAppendages

export StructureSimData, initappendagedata, setstructure, update_strstate

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

    state::AbstractVector{<:Union{AbstractVector, Real}}
    physicalstate::AbstractVector{<:Union{AbstractVector, Real}}
    controlinput::AbstractVector{<:Union{AbstractVector, Real}}
    disturbance::AbstractVector{<:Union{AbstractVector, Real}}

end

"""
    initappendagedata

initializer for the data container for structural simulation

# Arguments

* `model`: simulation model for the flexible appendage
* `initphysicalstate::Vector`: initial physical state value of the flexible appendage
* `datanum::Int`: numbers of the simulation data
"""
function initappendagedata(model, initphysicalstate::Vector, datanum::Int)::StructureSimData

    if typeof(model) == NoAppendagesModel

        # physical state vector (physical coordinate)
        physicalstate = [0.0 for _ in 1:datanum]

        # state vector (modal coordinate)
        state = [0.0 for _ in 1:datanum]

        # inputs
        controlinput = [0.0 for _ in 1:datanum]
        disturbance = [0.0 for _ in 1:datanum]

        return StructureSimData(state, physicalstate, controlinput, disturbance)
    else
        # physical state vector (physical coordinate)
        physicalstate = [zeros(SVector{model.dimstate}) for _ in 1:datanum]
        physicalstate[1] = SVector{model.dimstate}(initphysicalstate)

        # state vector (modal coordinate)
        state = [zeros(SVector{model.dimstate}) for _ in 1:datanum]
        state[1] = physicalstate2modalstate(model, initphysicalstate)

        # switch based on the dimension of the input
        if model.dimctrlinput == 1
            controlinput = [0.0 for _ in 1:datanum]
        else
            controlinput = [zeros(SVector{model.dimctrlinput}) for _ in 1:datanum]
        end

        if model.dimdistinput == 1
            disturbance = [0.0 for _ in 1:datanum]
        else
            disturbance = [zeros(SVector{model.dimdistinput}) for _ in 1:datanum]
        end

        return StructureSimData(state, physicalstate, controlinput, disturbance)
    end
end

"""
    setstructure

API function to define the model of the flexible appendages. Argument is a file path for the configuration file. This function is expected to be used directly with the configuration file

# Arguments

* `configfilepath::String`: path for the configuration file for the structural appendages
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
    setstructure

API function to define the model of the flexible appendages. Argument is the dictionary type variable

# Arguments

* `configdata::AbstractDict`: path for the configuration file for the structural appendages
"""
function setstructure(configdata::AbstractDict)

    if haskey(configdata, "modeling") == false
        throw(ErrorException("`modeling` is undefined in configuration"))
    end

    if configdata["modeling"] == "none"
        (structureparams, structuresimmodel) = NoAppendages.defmodel()
    elseif configdata["modeling"] == "spring-mass"
        (structureparams, structuresimmodel) = SpringMass.defmodel(configdata)
    else
        throw(ErrorException("No matching modeling method for the current configuration found. Possible typo in the configuration"))
    end

    if haskey(configdata, "disturbance")
        strdistconfig = setstrdistconfig(configdata["disturbance"])
    else
        throw(ErrorException("configuration for the disturbance input to the appendage structure is missing"))
    end

    return (structureparams, structuresimmodel, strdistconfig)
end

function update_strstate(strmodel, Ts, currenttime, currentstate, attiinput, strctrlinput, strdistinput)

    strmodeltype = typeof(strmodel)

    if strmodeltype == StateSpace
        StateSpace.updatestate(strmodel, Ts, currenttime, currentstate, attiinput, strctrlinput, strdistinput)

    elseif strmodeltype == NoAppendagesModel
        return 0
    end
end

end
