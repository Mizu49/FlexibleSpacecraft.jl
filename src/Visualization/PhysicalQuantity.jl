module PhysicalQuantity

using Plots

# Include module `DataContainers`
import ...DataContainers
using StaticArrays

export angularvelocities, eulerangles, quaternions

"""
    function angularvelocities(time::StepRangeLen, angularvelocity::Vector{StaticArrays.SVector{3, <:Real}}; timerange::Tuple{<:Real, <:Real} = (0, 0))::AbstractPlot

Plots angular velocity of each axis in one figure
"""
function angularvelocities(time::StepRangeLen, angularvelocity::Vector{StaticArrays.SVector{3, <:Real}}; timerange::Tuple{<:Real, <:Real} = (0, 0))::AbstractPlot

    plotlyjs()

    plt = plot();
    plt = plot!(plt, time, angularvelocity, 1, timerange = timerange, ylabelname = "Angular velocity (rad/s)", datalabel = "x-axis");
    plt = plot!(plt, time, angularvelocity, 2, timerange = timerange, ylabelname = "Angular velocity (rad/s)", datalabel = "y-axis");
    plt = plot!(plt, time, angularvelocity, 3, timerange = timerange, ylabelname = "Angular velocity (rad/s)", datalabel = "z-axis");

    return plt
end

"""
    eulerangles(time::StepRangeLen, eulerangle::Vector{StaticArrays.SVector{3, <:Real}}; timerange::Tuple{<:Real, <:Real} = (0, 0))::AbstractPlot

Plots time history of euler angles

"""
function eulerangles(time::StepRangeLen, eulerangle::Vector{StaticArrays.SVector{3, <:Real}}; timerange::Tuple{<:Real, <:Real} = (0, 0))::AbstractPlot

    plotlyjs()

    plt = plot();
    plt = plot!(plt, time, eulerangle, 1, timerange = timerange, ylabelname = "Euler angle (rad)", datalabel = "roll");
    plt = plot!(plt, time, eulerangle, 2, timerange = timerange, ylabelname = "Euler angle (rad)", datalabel = "pitch");
    plt = plot!(plt, time, eulerangle, 3, timerange = timerange, ylabelname = "Euler angle (rad)", datalabel = "yaw");

    return plt
end

"""
function quaternions(time::StepRangeLen, quaternion::Vector{StaticArrays.SVector{4, <:Real}}; timerange::Tuple{<:Real, <:Real} = (0, 0))

    Plot quaternions in single plot
"""
function quaternions(time::StepRangeLen, quaternion::Vector{StaticArrays.SVector{4, <:Real}}; timerange::Tuple{<:Real, <:Real} = (0, 0))

    plotlyjs()

    plt = plot();
    plt = plot!(plt, time, quaternion, 1, timerange = timerange, ylabelname = "Quaternion", datalabel = "q1");
    plt = plot!(plt, time, quaternion, 2, timerange = timerange, ylabelname = "Quaternion", datalabel = "q2");
    plt = plot!(plt, time, quaternion, 3, timerange = timerange, ylabelname = "Quaternion", datalabel = "q3");
    plt = plot!(plt, time, quaternion, 4, timerange = timerange, ylabelname = "Quaternion", datalabel = "q4");

    return plt
end

@recipe function f(time::StepRangeLen, quaternion::Vector{StaticArrays.SVector{4, <:Real}}, index::Integer; timerange = (0, 0), ylabelname = "No name", datalabel = "")
    if !(1 <= index <= 4)
        throw(BoundsError(quaternion[1], index))
    end

    xguide --> "Time (s)"
    yguide --> ylabelname
    label --> datalabel

    # get the index for data
    dataindex = DataContainers.getdataindex(timerange, convert(Float64, time.step))

    return time[dataindex], quaternion[dataindex, index]
end

@recipe function f(time::StepRangeLen, angularvelocity::Vector{StaticArrays.SVector{3, <:Real}}, axisindex::Integer; timerange = (0, 0), ylabelname = "No name", datalabel = "")

    xguide --> "Time (s)"
    yguide --> ylabelname
    label --> datalabel

    # get the index for data
    dataindex = DataContainers.getdataindex(timerange, convert(Float64, time.step))

    return time[dataindex], angularvelocity[dataindex, axisindex]
end

end
