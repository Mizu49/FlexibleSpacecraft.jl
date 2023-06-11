"""
    AppendagesBase

module for wrapping all the submodules for the structual dynamics for flexible spacecraft
"""
module AppendagesBase

using Reexport, YAML, StaticArrays
using ..UtilitiesBase, ..StructureDisturbance

export AppendagesInfo, AppendageData, initappendagedata, set_appendage_info, update_appendages!

# abstract types for the flexible appendages
abstract type AbstractAppendageParameters end
abstract type AbstractAppendageModel end
abstract type AbstractAppendageInternals end

include("DiscreteModeling.jl")
@reexport using .DiscreteModeling

"""
    AppendageData

struct of the data container for the states and inputs of the structural response of the flexible appendages
"""
struct AppendageData
    state::AbstractVector{<:Union{AbstractVector, Real}}
    physicalstate::AbstractVector{<:Union{AbstractVector, Real}}
    controlinput::AbstractVector{<:Union{AbstractVector, Real}}
    disturbance::AbstractVector{<:Union{AbstractVector, Real}}
end


"""
    AppendagesInfo

information of the flexible appendages
"""
struct AppendagesInfo
    params::Union{AbstractAppendageParameters, Nothing}
    model::Union{AbstractAppendageModel, Nothing}
    disturbance::Union{StructureDisturbance.AbstractAppendageDisturbance, Nothing}
end

"""
    set_appendage_info

API function to define the model of the flexible appendages. Argument is the dictionary type variable

# Arguments

* `configdata::AbstractDict`: path for the configuration file for the structural appendages
"""
function set_appendage_info(configdata::AbstractDict)::Union{AppendagesInfo, Nothing}

    if haskey(configdata, "modeling") == false
        throw(ErrorException("`modeling` is undefined in configuration"))
    end

    if configdata["modeling"] == "none"
        # flexible appendage does not exist
        return nothing

    elseif configdata["modeling"] == "spring-mass"
        # formulate spring-mass model of the flexible appendages

        (params, model) = DiscreteModeling.defmodel(configdata)

        # configure disturbance input to the flexible appendage
        if haskey(configdata, "disturbance")
            disturbance = setstrdistconfig(configdata["disturbance"])
        else
            throw(ErrorException("configuration for the disturbance input to the appendage structure is missing"))
        end

        return AppendagesInfo(params, model, disturbance)

    else
        throw(ErrorException("No matching modeling method for the current configuration found. Possible typo in the configuration"))
    end
end


"""
    initappendagedata

initializer for the data container for structural simulation
"""
function initappendagedata(info::AppendagesInfo, initphysicalstate::Vector, datanum::Int)

    # physical state vector (physical coordinate)
    physicalstate = [zeros(SVector{info.model.dimstate}) for _ in 1:datanum]
    physicalstate[1] = SVector{info.model.dimstate}(initphysicalstate)

    # state vector (modal coordinate)
    state = [zeros(SVector{info.model.dimstate}) for _ in 1:datanum]
    state[1] = physicalstate2modalstate(info.model, initphysicalstate)

    # switch based on the dimension of the input
    if info.model.dimctrlinput == 1
        controlinput = [0.0 for _ in 1:datanum]
    else
        controlinput = [zeros(SVector{info.model.dimctrlinput}) for _ in 1:datanum]
    end

    if info.model.dimdistinput == 1
        disturbance = [0.0 for _ in 1:datanum]
    else
        disturbance = [zeros(SVector{info.model.dimdistinput}) for _ in 1:datanum]
    end

    return AppendageData(state, physicalstate, controlinput, disturbance)
end


function update_appendages!(info::AppendagesInfo, Ts::Real, currenttime, currentstate, attitude2structure, strctrlinput, strdistinput)

    attiinput = attitude2structure.angularvelocity

    # time evolution
    nextstate = DiscreteModeling.updatestate(info.model, Ts, currenttime, currentstate, attiinput, strctrlinput, strdistinput)

    return nextstate
end


end
