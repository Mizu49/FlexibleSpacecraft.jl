module AttitudeVisualization

using GLMakie, LinearAlgebra

# Include module `DataContainers`
using ...DataContainers, StaticArrays

export plot_angularvelocity, plot_eulerangles, plot_quaternion, plot_angular_momentum

# labels for the XYZ axis
const XYZlabels = [
    "x-axis",
    "y-axis",
    "z-axis"
]

"""
    function plot_angularvelocity

Plots angular velocity of each axis in one figure
"""
function plot_angularvelocity(
    time::StepRangeLen,
    angularvelocity::AbstractVector{SVector{3, <:Real}};
    timerange::Tuple{<:Real, <:Real} = (0, 0)
    )

    # get the index for data
    dataindex = timerange2indexrange(timerange, time)

    fig = Figure();
    ax = Axis(
        fig[1, 1],
        xlabel = "Time (s)",
        ylabel = "Angular velocity (rad/s)"
    )

    for dim in 1:3
        lines!(
            ax, time[dataindex], angularvelocity[dataindex, dim],
            label = XYZlabels[dim]
        )
    end

    axislegend(ax)

    return fig
end

"""
    plot_eulerangles

Plots time history of euler angles
"""
function plot_eulerangles(
    time::StepRangeLen,
    eulerangle::AbstractVector{SVector{3, <:Real}};
    timerange::Tuple{<:Real, <:Real} = (0, 0)
    )

    # get the index for data
    dataindex = timerange2indexrange(timerange, time)

    fig = Figure();
    ax = Axis(
        fig[1, 1],
        xlabel = "Time (s)",
        ylabel = "Euler angle (rad)"
    )

    for dim in 1:3
        lines!(
            ax, time[dataindex], eulerangle[dataindex, dim],
            label = XYZlabels[dim]
        )
    end

    axislegend(ax)

    return fig
end

"""
    plot_quaternion

Plot plot_quaternion in single plot
"""
function plot_quaternion(
    time::StepRangeLen,
    quaternion::AbstractVector{SVector{4, <:Real}};
    timerange::Tuple{<:Real, <:Real} = (0, 0)
    )

    # get the index for data
    dataindex = timerange2indexrange(timerange, time)

    fig = Figure()
    ax = Axis(
        fig[1, 1],
        xlabel = "Time (s)",
        ylabel = "Quaternion (-)"
    )

    for dim in 1:4
        lines!(
            ax, time[dataindex], quaternion[dataindex, dim],
            label = "Quaternion $dim"
        )
    end

    axislegend(ax)

    return fig
end

"""
    plot_angular_momentum

plot time history of angular momentum
"""
function plot_angular_momentum(
    time::StepRangeLen,
    angular_momentum::AbstractVector{SVector{3, <:Real}};
    timerange::Tuple{<:Real, <:Real} = (0, 0)
    )

    # calculate norm of the angular momentum
    momentum_norm = norm.(angular_momentum)

    # get the index for data
    dataindex = timerange2indexrange(timerange, time)

    fig = Figure()
    ax = Axis(
        fig[1, 1],
        xlabel = "Time (s)",
        ylabel = "Angular momentum (kg⋅m^2⋅s^−1)"
    )

    # plot each dimension
    for dim in 1:3
        lines!(
            ax, time[dataindex], angular_momentum[dataindex, dim],
            label = XYZlabels[dim]
        )
    end

    # plot norm
    lines!(ax, time, momentum_norm, label = "Norm");

    axislegend(ax)

    return fig
end

end
