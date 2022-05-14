module FramePlot

using Plots
import ...DataContainers
using ...Frames

export dispframe, framegif

"""
    function dispframe(time, refCoordinate, coordinate)

Generates the 3D figure of body fixed frame
"""
function dispframe(time::Real, refCoordinate::Frame, coordinate::Frame)

    # Use `GR` backend
    gr()

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
function framegif(time::StepRangeLen, refframe::Frame, frames::Vector{<:Frame}; Tgif = 60, FPS = 3, timerange = (0, 0))

    # get the index for data
    dataindex = DataContainers.getdataindex(timerange, convert(Float64, time.step))

    Tsampling = convert(Float64, time.step)

    steps = round(Int, Tgif/Tsampling)

    # determine the index for animation for-loop
    if dataindex == Colon()
        animindex = 1:steps:size(time[dataindex], 1)
    else
        animindex = dataindex[1]:steps:dataindex[end]
    end

    # create animation
    anim = @animate for idx = animindex
        frame = getframe(time[idx], Tsampling, frames)
        dispframe(time[idx], refframe, frame)
    end

    # make gif image
    gifanime = gif(anim, "attitude.gif", fps = FPS)

    return gifanime
end

end
