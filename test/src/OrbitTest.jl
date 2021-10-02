using Test

include("../../src/FlexibleSpacecraft.jl")
using .FlexibleSpacecraft

# define a orbit info
orbitinfo = Orbit.OrbitInfo(
    Orbit.CircularOrbit(6370e+3 + 400e3, 3.986e+14),
    Orbit.OrbitalElements(111.8195, 51.6433, 421e3, 0.0001239, 241.3032, 212.0072),
    zeros(1,1)
)

angular_velocity = Orbit.get_angular_velocity(orbitinfo.orbitmodel)

speed = Orbit.get_velocity(orbitinfo.orbitmodel)

T = Orbit.get_timeperiod(orbitinfo.orbitmodel, unit = "minute")

# Earth-Centered frame (constant value)
ECI_frame = Frame(
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1]
)

orbitframe = Orbit.calc_orbitalframe(orbitinfo.orbitalelement, ECI_frame)

fig1 = PlotRecipe.dispframe(0, ECI_frame, orbitframe)

time = 0:60:T*60
data_num = round(Int, T*60/60) + 1;

# initialize orbit state data array
orbitdata = Orbit.initorbitdata(data_num, orbitframe)
RATframe = TimeLine.initframes(data_num, orbitframe)

@time for loopCounter = 0:data_num - 1

    orbitdata[loopCounter+1] = Orbit.updateorbitstate(orbitinfo.orbitalelement, orbitinfo.orbitmodel, time[loopCounter + 1])

    RATframe[loopCounter+1] = Orbit.update_radial_along_track(orbitframe, orbitinfo.orbitalelement, time[loopCounter+1], orbitdata.angularvelocity[loopCounter+1])

end

fig2 = PlotRecipe.frame_gif(time, 60, orbitframe, RATframe, Tgif = 90, FPS = 8)

display(fig1)
display(fig2)
