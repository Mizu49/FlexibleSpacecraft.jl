"""
    module PlotGenerator

module of functions that deal with plot of attitude dynamics
"""
module PlotGenerator

using Plots

function plotAngularVelocity()


    fig1 = plot(time, omegaBA[1, :], 
        xlabel ="Time [s]",
        ylabel ="omega [rad/s]",
        label  ="Angular velocity 1",
    )

    fig2 = plot(time, omegaBA[2, :], 
        xlabel ="Time [s]",
        ylabel ="omega [rad/s]",
        label  ="Angular velocity 2",
    )

    fig3 = plot(time, omegaBA[3, :], 
        xlabel ="Time [s]",
        ylabel ="omega [rad/s]",
        label  ="Angular velocity 3",
    )

    # Graph of the angular velocity
    figAngularVelocity = plot(fig1, fig2, fig3, layout = (3, 1), legend = true)


    display(figAngularVelocity)

    return figAngularVelocity
end

function plotCoordinate()

    coordFig = quiver(
        zeros(3), zeros(3), zeros(3),
        quiver = ( 
            [coordinateA.x[1], coordinateA.y[1], coordinateA.z[1]], 
            [coordinateA.x[2], coordinateA.y[2], coordinateA.z[2]], 
            [coordinateA.x[3], coordinateA.y[3], coordinateA.z[3]]),
        color = :black,
        linewidth = 4,
        xlims = (-1.2, 1.2),
        ylims = (-1.2, 1.2),
        zlims = (-1.2, 1.2),
        framestyle = :origin)

    coordFig = quiver!(
        zeros(3), zeros(3), zeros(3),
        quiver = ( 
            [coordinateB.x[1,plotIndex], coordinateB.y[1,plotIndex], coordinateB.z[1,plotIndex]], 
            [coordinateB.x[2,plotIndex], coordinateB.y[2,plotIndex], coordinateB.z[2,plotIndex]], 
            [coordinateB.x[3,plotIndex], coordinateB.y[3,plotIndex], coordinateB.z[3,plotIndex]]),
        color = :blue,
        linewidth = 4,
        xlims = (-1.2, 1.2),
        ylims = (-1.2, 1.2),
        zlims = (-1.2, 1.2),
        framestyle = :origin)

    display(coordFig)

    return coordFig
end

end
