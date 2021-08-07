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
ECI_frame = TimeLine.Coordinate(
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1]
)

C = Orbit.ECI2OrbitalPlaneFrame(elem)

orbit_frame = TimeLine.Coordinate(
    C * ECI_frame.x,
    C * ECI_frame.y,
    C * ECI_frame.z
)

PlotGenerator.dispframe(0, ECI_frame, orbit_frame)
