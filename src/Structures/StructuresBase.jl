"""
    StructuresBase

module for wrapping all the submodules for the structual dynamics for flexible spacecraft
"""
module StructuresBase

using Reexport, YAML, StaticArrays
using ..UtilitiesBase, ..StructureDisturbance

export AbstractAppendageModel, AppendageData, AppendageInternals, initappendagedata, setstructure, update_strstate!

# abstract types for the flexible appendages
abstract type AbstractAppendageParameters end
abstract type AbstractAppendageModel end
abstract type AbstractAppendageInternals end

include("SpringMass.jl")
@reexport using .SpringMass

"""
    AppendageData

struct of the data container for the states and inputs of the structural response of the flexible appendages

# Fields

* `state::AbstractVector{<:AbstractVector}`: data container for the state vector trajectory of the flexible appendage
* `physicalstate::AbstractVector{<:AbstractVector}`: data container for the physical displacement and velocity trajectory of the flexible appendage
* `controlinput::AbstractVector{<:AbstractVector}`: data container for the control input trajectory
* `disturbance::AbstractVector{<:AbstractVector}`: data container for the disturbance input trajectory
"""
struct AppendageData
    state::AbstractVector{<:Union{AbstractVector, Real}}
    physicalstate::AbstractVector{<:Union{AbstractVector, Real}}
    controlinput::AbstractVector{<:Union{AbstractVector, Real}}
    disturbance::AbstractVector{<:Union{AbstractVector, Real}}
end

"""
    AppendageInternals

internals of the appendage data
"""
mutable struct AppendageInternals<:AbstractAppendageInternals
    previousstate::AbstractVector{<:Real}
    currentstate::AbstractVector{<:Real}
    currentaccel::AbstractVector{<:Real}

    AppendageInternals(DOF::Integer) = begin

        initstate = SVector{2*DOF, Real}(zeros(2*DOF))
        initaccel = SVector{DOF, Real}(zeros(DOF))

        new(initstate, initstate, initaccel)
    end
end

"""
    initappendagedata

initializer for the data container for structural simulation

# Arguments

* `model`: simulation model for the flexible appendage
* `initphysicalstate::Vector`: initial physical state value of the flexible appendage
* `datanum::Int`: numbers of the simulation data
"""
function initappendagedata(model, initphysicalstate::Vector, datanum::Int)

    if model === nothing
        # nothing
        return nothing
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

        return AppendageData(state, physicalstate, controlinput, disturbance)
    end
end

"""
    AppendageInfo

information of the flexible appendages
"""
struct AppendageInfo
    params::AbstractAppendageParameters
    model::AbstractAppendageModel
    internals::AbstractAppendageInternals
    disturbance::StructureDisturbance.AbstractAppendageDisturbance
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

        params = nothing
        model = nothing
        internals = nothing
        disturbance = nothing

    elseif configdata["modeling"] == "spring-mass"
        (params, model) = SpringMass.defmodel(configdata)
        internals = AppendageInternals(2)

        # configure disturbance input to the flexible appendage
        if haskey(configdata, "disturbance")
            disturbance = setstrdistconfig(configdata["disturbance"])
        else
            throw(ErrorException("configuration for the disturbance input to the appendage structure is missing"))
        end
    else
        throw(ErrorException("No matching modeling method for the current configuration found. Possible typo in the configuration"))
    end

    return AppendageInfo(params, model, internals, disturbance)
end

function update_strstate!(strmodel::StateSpace, internals::AppendageInternals, Ts::Real, currenttime, currentstate, attiinput, strctrlinput, strdistinput)

    # time evolution
    nextstate = SpringMass.updatestate(strmodel, Ts, currenttime, currentstate, attiinput, strctrlinput, strdistinput)

    internals.previousstate = currentstate
    internals.currentstate = nextstate
    internals.currentaccel = (nextstate[(strmodel.DOF+1):end] - currentstate[(strmodel.DOF+1):end]) / Ts

    return nextstate
end

function update_strstate!(strmodel::Nothing, internals::AppendageInternals, Ts, currenttime, currentstate, attiinput, strctrlinput, strdistinput)

    return nothing
end


end
