"""
    AppendagesBase

module for wrapping all the submodules for the structual dynamics for flexible spacecraft
"""
module AppendagesBase

using YAML, StaticArrays
using ..UtilitiesBase

export AppendageData, initappendagedata, set_appendages_model, update_appendages!, AbstractAppendageParameters, AbstractAppendageModel

# abstract types for the flexible appendages
abstract type AbstractAppendageParameters end
abstract type AbstractAppendageModel end

include("DiscreteModeling.jl")
using .DiscreteModeling

"""
    AppendageData

struct of the data container for the states and inputs of the structural response of the flexible appendages
"""
struct AppendageData
    state::AbstractVector{<:Union{AbstractVector, Real}}
    physicalstate::AbstractVector{<:Union{AbstractVector, Real}}
end

"""
    set_appendages_model

API function to define the model of the flexible appendages. Argument is the dictionary type variable

# Arguments

* `configdata::AbstractDict`: path for the configuration file for the structural appendages
"""
function set_appendages_model(configdata::AbstractDict)

    if haskey(configdata, "modeling") == false
        throw(ErrorException("`modeling` is undefined in configuration"))
    end

    if configdata["modeling"] == "spring-mass"
        # formulate discrete spring-mass model of the flexible appendages
        (params, model) = DiscreteModeling.defmodel(configdata)
        return (params, model)
    else
        throw(ErrorException("No matching modeling method for the current configuration found. Possible typo in the configuration"))
    end
end

"""
    initappendagedata

initializer for the data container for structural simulation
"""
function initappendagedata(info::AbstractAppendageModel, initphysicalstate::Vector, datanum::Int)

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

    return AppendageData(state, physicalstate, controlinput)
end


function update_appendages!(model::AbstractAppendageModel, Ts::Real, currenttime, currentstate, attitude2structure, strctrlinput, strdistinput)

    attiinput = attitude2structure.angularvelocity

    # time evolution
    nextstate = DiscreteModeling.updatestate(model, Ts, currenttime, currentstate, attiinput, strctrlinput, strdistinput)

    return nextstate
end


end
