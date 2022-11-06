module FramePlot

using Plots, ProgressMeter, StaticArrays
using ...DataContainers, ...Frames

export dispframe, framegif

# RGB color setting for frame plots
axiscolor_x = RGB(colorant"#FF0000")
axiscolor_y = RGB(colorant"#008000")
axiscolor_z = RGB(colorant"#0000FF")
axiscolor_ref_x = RGB(colorant"#FFA5A5")
axiscolor_ref_y = RGB(colorant"#CCFECC")
axiscolor_ref_z = RGB(colorant"#A5A5FF")

"""
    function dispframe(time, refCoordinate, coordinate)

Generates the 3D figure of body fixed frame
"""
function dispframe(time::Real, refCoordinate::Frame, coordinate::Frame)

    # Use `GR` backend
    gr()

    # Plot of reference frame
    coordFig = plot()
    coordFig = _frame_vector!(refCoordinate.x, axiscolor_ref_x)
    coordFig = _frame_vector!(refCoordinate.y, axiscolor_ref_y)
    coordFig = _frame_vector!(refCoordinate.z, axiscolor_ref_z)

    # Plot of spacecraft fixed frame
    coordFig = _frame_vector!(coordinate.x, axiscolor_x)
    coordFig = _frame_vector!(coordinate.y, axiscolor_y)
    coordFig = _frame_vector!(coordinate.z, axiscolor_z)


    title!("Time: $time [s]")

    return coordFig
end

"""
    function frame_gif(time, Tsampling, refCoordinate, bodyCoordinateArray, Tgif = 0.4, FPS = 15)

Generates animation of frame rotation as GIF figure
"""
function framegif(time::StepRangeLen, refframe::Frame, frames::Vector{<:Frame}; Tgif = 60, FPS = 3, timerange = (0, 0))

    # get the index for data
    dataindex = timerange2indexrange(timerange, time)

    Tsampling = convert(Float64, time.step)

    steps = round(Int, Tgif/Tsampling)

    # determine the index for animation for-loop
    if dataindex == Colon()
        animindex = 1:steps:size(time[dataindex], 1)
    else
        animindex = dataindex[1]:steps:dataindex[end]
    end

    # create animation
    prog = Progress(length(animindex), 1, "Generating animation...", 50)   # progress meter
    anim = @animate for idx = animindex
        frame = getframe(time[idx], Tsampling, frames)
        dispframe(time[idx], refframe, frame)

        next!(prog) # update the progress meter
    end

    # make gif image
    gifanime = gif(anim, "attitude.gif", fps = FPS)

    return gifanime
end

function _frame_vector!(vec::SVector{3, <:Real}, argcolor; arglinewidth = 4)
    fig = quiver!(
        [0], [0], [0],
        quiver = (
            [vec[1]],
            [vec[2]],
            [vec[3]]),
        color = argcolor,
        linewidth = arglinewidth,
        framestyle = :origin)

    xlims!(-1.0, 1.0)
    ylims!(-1.0, 1.0)
    zlims!(-1.0, 1.0)

    return fig
end

end
