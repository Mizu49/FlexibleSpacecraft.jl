using Test

include("../../src/FlexibleSpacecraft.jl")
using .FlexibleSpacecraft

# Earth-Centered frame (constant value)
ECI_frame = Frame(
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1]
)

# define a orbit info
orbitinfo = Orbit.OrbitInfo(Orbit.OrbitalElements(111.8195, 51.6433, 421e3, 0.0001239, 241.3032, 212.0072), ECI_frame)

angular_velocity = Orbit.get_angular_velocity(orbitinfo.orbitmodel)

speed = Orbit.get_velocity(orbitinfo.orbitmodel)

T = Orbit.get_timeperiod(orbitinfo.orbitmodel, unit = "minute")

fig1 = PlotRecipe.dispframe(0, ECI_frame, orbitinfo.planeframe)

time = 0:60:T*60
data_num = round(Int, T*60/60) + 1;

# initialize orbit state data array
orbitdata = Orbit.initorbitdata(data_num, orbitinfo.planeframe)
RATframe = TimeLine.initframes(data_num, orbitinfo.planeframe)

@time for loopCounter = 0:data_num - 1

    orbitdata[loopCounter+1] = Orbit.updateorbitstate(orbitinfo.orbitalelement, orbitinfo.orbitmodel, time[loopCounter + 1])

    RATframe[loopCounter+1] = Orbit.update_radial_along_track(orbitinfo.planeframe, orbitinfo.orbitalelement, time[loopCounter+1], orbitdata.angularvelocity[loopCounter+1])

end

fig2 = PlotRecipe.frame_gif(time, 60, orbitinfo.planeframe, RATframe, Tgif = 90, FPS = 8)

display(fig1)
display(fig2)
