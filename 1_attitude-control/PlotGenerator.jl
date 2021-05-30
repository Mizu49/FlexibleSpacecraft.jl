"""
    module PlotGenerator

module of functions that deal with plot of attitude dynamics
"""
module PlotGenerator

using Plots

# Include module `TimeLine`
include("TimeLine.jl")
import .TimeLine

"""
    plotAngularVelocity(time::Matrix, angularVelocity::Matrix)
"""
function plotAngularVelocity(time, angularVelocity)


    fig1 = plot(time, angularVelocity[1, :], 
        xlabel ="Time [s]",
        ylabel ="omega [rad/s]",
        label  ="Angular velocity 1",
    );

    fig2 = plot(time, angularVelocity[2, :], 
        xlabel ="Time [s]",
        ylabel ="omega [rad/s]",
        label  ="Angular velocity 2",
    );

    fig3 = plot(time, angularVelocity[3, :], 
        xlabel ="Time [s]",
        ylabel ="omega [rad/s]",
        label  ="Angular velocity 3",
    );

    # Graph of the angular velocity
    figAngularVelocity = plot(fig1, fig2, fig3, layout = (3, 1), legend = true);    

    return figAngularVelocity
end

"""
    plotCoordinate(timeIndex::Int, refCoordinate, bodyCoordinate)
"""
function plotCoordinate(time, refCoordinate, bodyCoordinate)

    coordFig = quiver(
        zeros(3), zeros(3), zeros(3),
        quiver = ( 
            [refCoordinate.x[1], refCoordinate.y[1], refCoordinate.z[1]], 
            [refCoordinate.x[2], refCoordinate.y[2], refCoordinate.z[2]], 
            [refCoordinate.x[3], refCoordinate.y[3], refCoordinate.z[3]]),
        color = RGBA(colorant"#000000", 0.5),
        linewidth = 4,
        xlims = (-1.0, 1.0),
        ylims = (-1.0, 1.0),
        zlims = (-1.0, 1.0),
        framestyle = :origin)

    coordFig = quiver!(
        zeros(3), zeros(3), zeros(3),
        quiver = ( 
            [bodyCoordinate.x[1], bodyCoordinate.y[1], bodyCoordinate.z[1]], 
            [bodyCoordinate.x[2], bodyCoordinate.y[2], bodyCoordinate.z[2]], 
            [bodyCoordinate.x[3], bodyCoordinate.y[3], bodyCoordinate.z[3]]),
        color = :blue,
        linewidth = 4,
        xlims = (-1.0, 1.0),
        ylims = (-1.0, 1.0),
        zlims = (-1.0, 1.0),
        framestyle = :origin,
        title = "Time: $time [s]")

    return coordFig
end

"""
    function getCoordinateGif(time, Tsampling, refCoordinate, bodyCoordinateArray, Tgif, Fps,)
"""
function getCoordinateGif(time, Tsampling, refCoordinate, bodyCoordinateArray, Tgif = 0.4, FPS = 15)

    dataNum = size(time, 1)
    steps = round(Int, Tgif/Tsampling)

    # create animation
    anim = @animate for index in 1:dataNum

        bodyCoordinate = TimeLine.extractCoordinateVector(time[index], Tsampling, bodyCoordinateArray)

        plotCoordinate(time[index], refCoordinate, bodyCoordinate)
    end every steps

    # make gif image
    gifanime = gif(anim, "attitude.gif", fps = FPS)

    return gifanime
end

end