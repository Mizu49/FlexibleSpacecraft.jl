module PhysicalQuantity

using Plots

# Include module `TimeLine`
import ...TimeLine
using StaticArrays

export angular_velocity

"""
    angular_velocity(time, angularVelocity)

Generator plots of angular velocity in each axis
"""
function angular_velocity(time::StepRangeLen, angularVelocity::Vector{StaticArrays.SVector{3, Float64}})

    # Use `PlotlyJS` backend
    plotlyjs()

    fig1 = plot(time, getindex.(angularVelocity, 1),
        xlabel ="Time [s]",
        ylabel ="omega [rad/s]",
        label  ="Angular velocity 1",
    );

    fig2 = plot(time, getindex.(angularVelocity, 2),
        xlabel ="Time [s]",
        ylabel ="omega [rad/s]",
        label  ="Angular velocity 2",
    );

    fig3 = plot(time, getindex.(angularVelocity, 3),
        xlabel ="Time [s]",
        ylabel ="omega [rad/s]",
        label  ="Angular velocity 3",
    );

    # Graph of the angular velocity
    fig_angular_velocity = plot(fig1, fig2, fig3, layout = (3, 1), legend = true);

    return fig_angular_velocity
end

end
