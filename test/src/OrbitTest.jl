using Test

include("../../src/FlexibleSpacecraft.jl")
using .FlexibleSpacecraft

# define a circular orbit info
circular_orbit = Orbit.CircularOrbit(6370e+3 + 400e3, 3.986e+14)

angular_velocity = Orbit.get_angular_velocity(circular_orbit)

speed = Orbit.get_velocity(circular_orbit)

T = Orbit.get_timeperiod(circular_orbit, unit = "minute")

# Orbital elements of ISS
elem = Orbit.OrbitalElements(111.8195, 51.6433, 421e3, 0.0001239, 241.3032, 212.0072)

# Earth-Centered frame (constant value)
ECI_frame = TimeLine.Frame(
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1]
)

orbitframe = Orbit.calc_orbitalframe(elem, ECI_frame)

fig1 = PlotRecipe.dispframe(0, ECI_frame, orbitframe)

time = 0:60:T*60
data_num = round(Int, T*60/60) + 1;
spacecraft_RAT = TimeLine.initframes(data_num, orbitframe)

for loopCounter = 0:data_num - 1

    spacecraft_RAT[loopCounter + 1] = Orbit.update_radial_along_track(orbitframe, elem, time[loopCounter + 1], angular_velocity)

end

fig2 = PlotRecipe.frame_gif(time, 60, orbitframe, spacecraft_RAT, Tgif = 90, FPS = 8)

display(fig1)
display(fig2)
