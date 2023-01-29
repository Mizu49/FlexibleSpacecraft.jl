module FramePlot

using GLMakie, ProgressMeter, StaticArrays, ColorTypes
using ...DataContainers, ...Frames

export animate_attitude

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
function _Frame2Arrows(frame)

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
    _plot_frame!
"""
function _plot_frame!(ax, ref_x, ref_y, ref_z, color_config)

    arrows!(
        ax,
        zeros(3), zeros(3), zeros(3),
        ref_x, ref_y, ref_z,
        linewidth = 0.05,
        color = color_config
    )

    return
end

"""
    frame_gif

Generates animation of frame rotation as GIF figure
"""
function animate_attitude(
    time::StepRangeLen,
    refframe::Frame,
    frames::Vector{<:Frame};
    Tgif = 1e-1,
    FPS = 20,
    timerange = (0, 0)
    )

    # extract indeces of data to be plotted
    Tsampling = convert(Float64, time.step)
    steps = round(Int, Tgif/Tsampling)
    dataindex = timerange2indexrange(timerange, time)
    if dataindex == Colon()
        animindex = 1:steps:size(time[dataindex], 1)
    else
        animindex = dataindex[1]:steps:dataindex[end]
    end

    # vectors for the corrdinate frame
    (ref_x, ref_y, ref_z) = _Frame2Arrows(refframe)
    (BRF_x, BRF_y, BRF_z) = _Frame2Arrows(frames[animindex[1]])

    # set observables
    obs_x = Observable(BRF_x)
    obs_y = Observable(BRF_y)
    obs_z = Observable(BRF_z)

    # create instance
    fig = Figure(; resolution = (600, 600))
    ax = Axis3(
        fig[1, 1],
        xlabel = "x label",
        ylabel = "y label",
        zlabel = "z label",
        elevation = pi/6,
        azimuth   = pi/4,
        aspect = :data,
        viewmode = :fit,
        limits = (-1.0, 1.0, -1.0, 1.0, -1.0, 1.0)
    )
    hidespines!(ax)
    hidedecorations!(ax)

    # plot frame vectors
    _plot_frame!(ax, ref_x, ref_y, ref_z, refaxiscolors)
    _plot_frame!(ax, obs_x, obs_y, obs_z, axiscolors)

    # create animation
    prog = Progress(length(animindex), 1, "Generating animation...", 20) # progress meter
    record(fig, "attitude.mp4", animindex; framerate = FPS) do idx

        # update values
        (BRF_x, BRF_y, BRF_z) = _Frame2Arrows(frames[idx])

        # update observables
        obs_x[] = BRF_x
        obs_y[] = BRF_y
        obs_z[] = BRF_z

        ax.title = "Time: $(time[idx]) (s)"

        # update progress meter
        next!(prog)
    end

end

end
