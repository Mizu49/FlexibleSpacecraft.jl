"""
    module TimeLine

module of time line of the physical quantity of spacecraft attitude dynamics

"""
module TimeLine

using StructArrays
using StaticArrays
using ..Frames

export InitData, initsimulationdata, timeindex

"""
    function init_angular_velocity_array(simdata_num, initital_value::Vector)

Initialize array that contains time response of angular velocity
"""
function initangularvelocity(datanum, initialvelocity::SVector{3, <:Real})

    angularvelocitydata = [SVector(0.0, 0.0, 0.0) for _ in 1:datanum]
    angularvelocitydata[1] = initialvelocity

    return angularvelocitydata
end


"""
    function init_quaternion_array(simdata_num, initial_value::Vector[4])

initialize array that contains time response of quaternion
"""
function initquaterniondata(datanum, initialvalue::SVector{4, <:Real})

    quaterniondata = [SVector(0.0, 0.0, 0.0, 0.0) for _ in 1:datanum]
    quaterniondata[1] = initialvalue

    return quaterniondata
end

"""
    Base.getindex(v::Vector{<:SVector}, r::AbstractRange, datarow::Int)

get a 1-D subset of the every `datarow`-th row of `v::Vector{<:SVector}` within `r::AbstractRange`, used for custom data container for `FlexibleSpacecraft.jl`
"""
function Base.getindex(v::Vector{<:SVector}, r::Union{AbstractRange, Colon}, datarow::Int)
    return getindex.(v[r], datarow)
end

"""
    Base.getindex(v::Vector{<:SVector}, r::Int, datarow::Int)

get an element of the `v<:SVector`, used for custom data container for `FlexibleSpacecraft.jl`
"""
function Base.getindex(v::Vector{<:SVector}, r::Int, datarow::Int)
    return getindex(v[r], datarow)
end

"""
    struct InitiData

Struct that consists of the initial state value of the time-variant physical amounts in simulation
"""
struct InitData

    # Spacecraft state variable
    quaternion::SVector{4, Real}
    angularvelocity::SVector{3, Real}
    bodyframe::Frame

end

"""
    initsimulationdata(datanum::Int, initialdata::InitData)

Initialize the data container for the attitude dynamics
"""
function initsimulationdata(datanum::Int, initialdata::InitData)

    return StructArray(
        quaternion = initquaterniondata(datanum, initialdata.quaternion),
        angularvelocity = initangularvelocity(datanum, initialdata.angularvelocity),
        bodyframe = initframes(datanum, initialdata.bodyframe)
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
        startindex = convert(Int64, round(timerange[1]/samplingtime) + 1)
        endindex = convert(Int64, round(timerange[2]/samplingtime) + 1)

        return startindex:endindex
    end
end

end
