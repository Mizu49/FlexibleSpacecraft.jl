module AppendagesVisualization

using Plots, StaticArrays
using ...DataContainers

export plot_physicalstate

function plot_physicalstate(time::AbstractRange, physicalstate::AbstractVector{<:SVector})::AbstractPlot

    # get information
    dimstate = size(physicalstate[1], 1)
    DOF = Integer(dimstate / 2)

    # backend
    plotlyjs()

    fig = plot()
    for idx = 1:DOF
        fig = plot!(
            time,
            physicalstate[:, idx],
            label = "x_$idx",
            linewidth = 2,
            xguide = "Time (s)",
            yguide = "Displacement (m)",
            guidefontsize = 15
        )
    end

    return fig
end

end
