"""
    module PlotRecipe

module of functions that show us the beautiful figures of spacecraft attitude dynamics
"""
module PlotRecipe

using Plots

# Include module `TimeLine`
include("TimeLine.jl")
import .TimeLine

"""
    angular_velocity(time, angularVelocity)

Generator plots of angular velocity in each axis
"""
function angular_velocity(time, angularVelocity)

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
    fig_angular_velocity = plot(fig1, fig2, fig3, layout = (3, 1), legend = true);

    return fig_angular_velocity
end

"""
    function dispframe(time, refCoordinate, coordinate)

Generates the 3D figure of body fixed frame
"""
function dispframe(time, refCoordinate, coordinate)

    # Plot of reference frame
    coordFig = quiver(
        [0], [0], [0],
        quiver = (
            [refCoordinate.x[1]],
            [refCoordinate.x[2]],
            [refCoordinate.x[3]]),
        color = RGB(colorant"#FFA5A5"),
        linewidth = 2,)

    coordFig = quiver!(
        [0], [0], [0],
        quiver = (
            [refCoordinate.y[1]],
            [refCoordinate.y[2]],
            [refCoordinate.y[3]]),
        color = RGB(colorant"#CCFECC"),
        linewidth = 2,)

    coordFig = quiver!(
        [0], [0], [0],
        quiver = (
            [refCoordinate.z[1]],
            [refCoordinate.z[2]],
            [refCoordinate.z[3]]),
        color = RGB(colorant"#A5A5FF"),
        linewidth = 2,)

    # Plot of spacecraft fixed frame
    coordFig = quiver!(
        [0], [0], [0],
        quiver = (
            [coordinate.x[1]],
            [coordinate.x[2]],
            [coordinate.x[3]]),
        color = RGB(colorant"#FF0000"),
        linewidth = 4,)

    coordFig = quiver!(
        [0], [0], [0],
        quiver = (
            [coordinate.y[1]],
            [coordinate.y[2]],
            [coordinate.y[3]]),
        color = RGB(colorant"#008000"),
        linewidth = 4,)

    coordFig = quiver!(
        [0], [0], [0],
        quiver = (
            [coordinate.z[1]],
            [coordinate.z[2]],
            [coordinate.z[3]]),
        color = RGB(colorant"#0000FF"),
        linewidth = 4,
        framestyle = :origin)

    xlims!(-1.0, 1.0)
    ylims!(-1.0, 1.0)
    zlims!(-1.0, 1.0)
    title!("Time: $time [s]")

    return coordFig
end

"""
    function frame_gif(time, Tsampling, refCoordinate, bodyCoordinateArray, Tgif = 0.4, FPS = 15)

Generates animation of frame rotation as GIF figure
"""
function frame_gif(time, Tsampling, refCoordinate, bodyCoordinateArray; Tgif = 60, FPS = 3)

    dataNum = size(time, 1)
    steps = round(Int, Tgif/Tsampling)

    # create animation
    anim = @animate for index in 1:dataNum

        bodyCoordinate = TimeLine.getframe(time[index], Tsampling, bodyCoordinateArray)

        dispframe(time[index], refCoordinate, bodyCoordinate)
    end every steps

    # make gif image
    gifanime = gif(anim, "attitude.gif", fps = FPS)

    return gifanime
end

end
