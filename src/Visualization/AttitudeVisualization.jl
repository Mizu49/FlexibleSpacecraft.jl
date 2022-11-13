module AttitudeVisualization

using Plots

# Include module `DataContainers`
using ...DataContainers, StaticArrays

export plot_angularvelocity, plot_eulerangles, plot_quaternion

"""
    function plot_angularvelocity(time::StepRangeLen, angularvelocity::Vector{StaticArrays.SVector{3, <:Real}}; timerange::Tuple{<:Real, <:Real} = (0, 0))::AbstractPlot

Plots angular velocity of each axis in one figure
"""
function plot_angularvelocity(
    time::StepRangeLen,
    angularvelocity::AbstractVector{SVector{3, <:Real}};
    timerange::Tuple{<:Real, <:Real} = (0, 0)
    )::AbstractPlot

    plotlyjs()

    # using plot recipe for these plots
    plt = plot();
    plt = plot!(plt, time, angularvelocity, 1, timerange = timerange, ylabelname = "Angular velocity (rad/s)", datalabel = "x-axis");
    plt = plot!(plt, time, angularvelocity, 2, timerange = timerange, ylabelname = "Angular velocity (rad/s)", datalabel = "y-axis");
    plt = plot!(plt, time, angularvelocity, 3, timerange = timerange, ylabelname = "Angular velocity (rad/s)", datalabel = "z-axis");

    return plt
end

"""
    plot_eulerangles(time::StepRangeLen, eulerangle::Vector{StaticArrays.SVector{3, <:Real}}; timerange::Tuple{<:Real, <:Real} = (0, 0))::AbstractPlot

Plots time history of euler angles

"""
function plot_eulerangles(
    time::StepRangeLen,
    eulerangle::AbstractVector{SVector{3, <:Real}};
    timerange::Tuple{<:Real, <:Real} = (0, 0)
    )::AbstractPlot

    plotlyjs()

    # using type recipe
    plt = plot();
    plt = plot!(plt, time, eulerangle, 1, timerange = timerange, ylabelname = "Euler angle (rad)", datalabel = "roll");
    plt = plot!(plt, time, eulerangle, 2, timerange = timerange, ylabelname = "Euler angle (rad)", datalabel = "pitch");
    plt = plot!(plt, time, eulerangle, 3, timerange = timerange, ylabelname = "Euler angle (rad)", datalabel = "yaw");

    return plt
end

"""
function plot_quaternion(time::StepRangeLen, quaternion::Vector{StaticArrays.SVector{4, <:Real}}; timerange::Tuple{<:Real, <:Real} = (0, 0))

    Plot plot_quaternion in single plot
"""
function plot_quaternion(
    time::StepRangeLen,
    quaternion::AbstractVector{SVector{4, <:Real}};
    timerange::Tuple{<:Real, <:Real} = (0, 0)
    )

    plotlyjs()

    # using type recipe
    plt = plot();
    plt = plot!(plt, time, quaternion, 1, timerange = timerange, ylabelname = "Quaternion", datalabel = "q1");
    plt = plot!(plt, time, quaternion, 2, timerange = timerange, ylabelname = "Quaternion", datalabel = "q2");
    plt = plot!(plt, time, quaternion, 3, timerange = timerange, ylabelname = "Quaternion", datalabel = "q3");
    plt = plot!(plt, time, quaternion, 4, timerange = timerange, ylabelname = "Quaternion", datalabel = "q4");

    return plt
end

# type recipe for data container of quaternion
@recipe function f(
    time::StepRangeLen,
    quaternion::AbstractVector{SVector{4, <:Real}},
    axisindex::Integer;
    timerange = (0, 0),
    ylabelname = "no y-label",
    datalabel = "no data label")


    # check the bounds for indexing
    if !(1 <= axisindex <= 4)
        throw(BoundsError(quaternion[1], index))
    end

    xguide --> "Time (s)"
    yguide --> ylabelname
    label --> datalabel
    linewidth --> 2
    fontfamily --> "Times"
    guidefontsize --> 15
    tickfontsize --> 15
    legendfontsize --> 15
    titlefontsize --> 15

    # get the index for data
    dataindex = timerange2indexrange(timerange, time)

    return time[dataindex], quaternion[dataindex, axisindex]
end

# type recipe for data container of vector of 3D vectors, mainly for the angles
@recipe function f(
    time::StepRangeLen,
    anglevectors::AbstractVector{SVector{3, <:Real}},
    axisindex::Integer;
    timerange = (0, 0),
    ylabelname = "no y-label",
    datalabel = "no data label")

    # check the bounds for indexing
    if !(1 <= axisindex <= 3)
        throw(BoundsError(anglevectors[1], index))
    end

    xguide --> "Time (s)"
    yguide --> ylabelname
    label --> datalabel
    linewidth --> 2
    fontfamily --> "Times"
    guidefontsize --> 15
    tickfontsize --> 15
    legendfontsize --> 15
    titlefontsize --> 15

    # get the index for data
    dataindex = timerange2indexrange(timerange, time)

    return time[dataindex], anglevectors[dataindex, axisindex]
end

end
