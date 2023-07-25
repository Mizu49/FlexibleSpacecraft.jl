module FramePlot

using GLMakie, ProgressMeter, StaticArrays, ColorTypes, LinearAlgebra
using ...UtilitiesBase, ...DataContainers, ...Frames, ...KinematicsBase, ...OrbitBase

export animate_attitude

include("spacecraft_body.jl")

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

const T = SMatrix{3, 3}(diagm([1.0, -1.0, -1.0]))

function _extract_data(simdata)

    time = simdata.time

    C_ECI2BRF = simdata.attitude.C_ECI2BRF

    if isnothing(simdata.orbit)
        C_ECI2LVLH = nothing
    else
        C_ECI2LVLH = simdata.orbit.C_ECI2LVLH
    end

    return (time, C_ECI2BRF, C_ECI2LVLH)
end

"""
    frame_gif

Generates animation of frame rotation as GIF figure
"""
function animate_attitude(
    simdata;
    Tgif = 1e-1,
    FPS = 20,
    timerange = (0, 0),
    filename = "attitude.gif",
    elevation = pi/6,
    azimuth   = pi/4
    )

    # extract necessary data
    (time, C_ECI2BRF, C_ECI2LVLH) = _extract_data(simdata)

    # extract indeces of data to be plotted
    Tsampling = convert(Float64, time.step)
    steps = round(Int, Tgif/Tsampling)
    dataindex = timerange2indexrange(timerange, time)
    if dataindex == Colon()
        animindex = 1:steps:size(time[dataindex], 1)
    else
        animindex = dataindex[1]:steps:dataindex[end]
    end

    # initialize vectors for the corrdinate frame
    (BRF_x, BRF_y, BRF_z) = _Frame2Arrows(LVLHUnitFrame)

    # spacecraft body polygon
    spacecraft = get_spacecraft_polygon()

    # set observables
    obs_x = Observable(BRF_x)
    obs_y = Observable(BRF_y)
    obs_z = Observable(BRF_z)
    obs_spacecraft_points = Observable(spacecraft.points)

    # create instance
    fig = Figure(; resolution = (600, 600))
    ax = Axis3(
        fig[1, 1],
        xlabel = "x label",
        ylabel = "y label",
        zlabel = "z label",
        elevation = elevation,
        azimuth   = azimuth,
        aspect = :data,
        viewmode = :fit,
        limits = (-1.5, 1.5, -1.5, 1.5, -1.5, 1.5)
    )
    hidespines!(ax)
    hidedecorations!(ax)

    # plot frame vectors
    _plot_frame!(ax, obs_x, obs_y, obs_z, axiscolors)

    # plot spacecraft body
    mesh!(ax, obs_spacecraft_points, spacecraft.faces, color = :yellow ,shading = true)

    # directions
    text!(
        label_positions,
        text = label_texts,
        align = (:center, :center),
    )
    lines!(ax, [-1.5, 1.5], [0.0, 0.0], [0.0, 0.0], linestyle = :dash, color = :black, linewidth = 3)
    lines!(ax, [0.0, 0.0], [-1.5, 1.5], [0.0, 0.0], linestyle = :dash, color = :black, linewidth = 3)
    lines!(ax, [0.0, 0.0], [0.0, 0.0], [-1.5, 1.5], linestyle = :dash, color = :black, linewidth = 3)

    # create animation
    prog = Progress(length(animindex), 1, "Animating...", 20) # progress meter
    record(fig, filename, animindex; framerate = FPS) do idx

        # update attitude vectors
        if isnothing(C_ECI2LVLH)
            C_LVLH2BRF = T * transpose(C_ECI2BRF[idx])
        else
            # C_LVLH2BRF =  transpose(C_ECI2LVLH[idx]) * transpose(C_ECI2BRF[idx])
            C_LVLH2BRF = T * transpose(C_ECI2BRF[idx] * transpose(C_ECI2LVLH[idx]))
        end
        BRF = C_LVLH2BRF * UnitFrame
        (BRF_x, BRF_y, BRF_z) = _Frame2Arrows(BRF)

        # calculate spacecraft points are defined in the LVLH frame
        spacecraft_points = C_LVLH2BRF * spacecraft.points

        # update observables
        obs_x[] = BRF_x
        obs_y[] = BRF_y
        obs_z[] = BRF_z
        obs_spacecraft_points[] = spacecraft_points

        ax.title = "Time: $(time[idx]) (s)"

        # update progress meter
        next!(prog)
    end

end

end
