"""
    module TimeLine

module of time line of the physical quantity of spacecraft attitude dynamics

"""
module TimeLine

using StaticArrays, StructArrays
using ..Frames

export AttitudeData, InitData, initsimulationdata, timeindex

"""
    function _initangularvelocity(simdata_num, initital_value::Vector)

Initialize array that contains time response of angular velocity
"""
function _initangularvelocity(datanum, initialvelocity::SVector{3, <:Real})

    angularvelocitydata = [SVector(0.0, 0.0, 0.0) for _ in 1:datanum]
    angularvelocitydata[1] = initialvelocity

    return angularvelocitydata
end

"""
    function _initquaternion(simdata_num, initial_value::Vector[4])

initialize array that contains time response of quaternion
"""
function _initquaternion(datanum, initialvalue::SVector{4, <:Real})

    quaterniondata = [SVector(0.0, 0.0, 0.0, 0.0) for _ in 1:datanum]
    quaterniondata[1] = initialvalue

    return quaterniondata
end

"""
    struct InitData

Struct that consists of the initial state value of the time-variant physical amounts in simulation
"""
struct InitData

    # Spacecraft state variable
    quaternion::SVector{4, Real}
    angularvelocity::SVector{3, Real}
    bodyframe::Frame

end

struct AttitudeData
    datanum::Int
    quaternion::Vector{SVector{4, <:Real}}
    angularvelocity::Vector{SVector{3, <:Real}}
    bodyframe::StructArray
    RPYframe::StructArray
    eulerangle::Vector{SVector{3, <:Real}}
end

"""
    initsimulationdata(datanum::Int, initialdata::InitData)

Initialize the data container for the attitude dynamics
"""
function initsimulationdata(datanum::Int, initialdata::InitData)

    return AttitudeData(
        datanum,
        _initquaternion(datanum, initialdata.quaternion),
        _initangularvelocity(datanum, initialdata.angularvelocity),
        initframes(datanum, initialdata.bodyframe),
        initframes(datanum, initialdata.bodyframe),
        [SVector{3}(0.0, 0.0, 0.0) for _ in 1:datanum]
    )
end

"""
    function getdataindex(timerange::Tuple{<:Real, <:Real}, samplingtime::Real)::Union{UnitRange{Int64}, Colon}

returns an `index::::Union{UnitRange{Int64}, Colon}` that corresponding to the given `timerange::Tuple{<:Real, <:Real}`
"""
function getdataindex(timerange::Tuple{<:Real, <:Real}, samplingtime::Real)::Union{UnitRange{Int64}, Colon}

    if timerange == (0, 0)
        return :;
    else
        if timerange[1] >= timerange[2]
            throw(DomainError(timerange, "setting for `timerange` is invalid"))
        end

        startindex = convert(Int64, round(timerange[1]/samplingtime) + 1)
        endindex = convert(Int64, round(timerange[2]/samplingtime) + 1)

        return startindex:endindex
    end
end

end
