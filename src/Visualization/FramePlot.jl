module FramePlot

using GLMakie, ProgressMeter, StaticArrays, ColorTypes
using ...DataContainers, ...Frames

export dispframe!, framegif

# RGB color setting for frame plots
const axiscolors = [
    RGBA(1.0, 0.0, 0.0, 1),
    RGBA(0.0, 1.0, 0.0, 1),
    RGBA(0.0, 0.0, 1.0, 1)
]

const refaxiscolors = [
    RGBA(1.0, 0.0, 0.0, 0.5),
    RGBA(0.0, 1.0, 0.0, 0.5),
    RGBA(0.0, 0.0, 1.0, 0.5)
]


"""
    _Frame2Arrows
"""
function _Frame2Arrows(frame::Frame)

    dir_x = [
        frame.x[1]
        frame.y[1]
        frame.z[1]
    ]

    dir_y = [
        frame.x[2]
        frame.y[2]
        frame.z[2]
    ]

    dir_z = [
        frame.x[3]
        frame.y[3]
        frame.z[3]
    ]

    return (dir_x, dir_y, dir_z)
end


"""
    function dispframe

Generates the 3D figure of body fixed frame
"""
function dispframe!(fig, time::Real, refCoordinate::Frame, coordinate::Frame)

    ax = Axis3(
        fig[1, 1],
        title  = "Time: $time (s)",
        xlabel = "x label",
        ylabel = "y label",
        zlabel = "z label",
        # elevation = pi/4,
        # azimuth   = pi/4,
        aspect = :data,
        viewmode = :fit,
        limits = (-1.0, 1.0, -1.0, 1.0, -1.0, 1.0)
    )

    (ref_x, ref_y, ref_z) = _Frame2Arrows(refCoordinate)
    (BRF_x, BRF_y, BRF_z) = _Frame2Arrows(coordinate)

    arrows!(
        ax,
        zeros(3), zeros(3), zeros(3),
        ref_x, ref_y, ref_z,
        color = refaxiscolors
    )

    arrows!(
        ax,
        zeros(3), zeros(3), zeros(3),
        BRF_x, BRF_y, BRF_z,
        color = axiscolors
    )

    hidespines!(ax)
    hidedecorations!(ax)

    return
end

"""
    frame_gif

Generates animation of frame rotation as GIF figure
"""
function framegif(time::StepRangeLen, refframe::Frame, frames::Vector{<:Frame}; Tgif = 60, FPS = 3, timerange = (0, 0))

end

end
