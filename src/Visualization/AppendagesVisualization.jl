module AppendagesVisualization

using GLMakie, StaticArrays
using ...DataContainers

export plot_physicalstate

function plot_physicalstate(
    time::AbstractRange,
    physicalstate::AbstractVector{<:SVector};
    timerange = (0, 0)
    )

    # get the index for data
    dataindex = timerange2indexrange(timerange, time)

    # get information
    dimstate = size(physicalstate[1], 1)
    DOF = Integer(dimstate / 2)

    fig = Figure()
    ax = Axis(
        fig[1, 1],
        xlabel = "Time (s)",
        ylabel = "Displacement (m)"
    )
    for idx = 1:DOF
        lines!(
            ax,
            time[dataindex],
            physicalstate[dataindex, idx],
            label = "x$idx",
        )
    end

    axislegend(ax)

    return fig
end

end
