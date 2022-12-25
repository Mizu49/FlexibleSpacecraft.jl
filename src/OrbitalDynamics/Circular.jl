module Circular

using ..OrbitBase, ..Elements

export CircularOrbit, get_velocity, get_timeperiod

"""
    struct CircularOrbit(_radius::Float64, _gravityconstant::Float64)

Parameters of a circular orbit

## Arguments

* `_radius`: radius of orbit (m)
* `_gravityconstant`: standard gravitational parameter ``μ = GM``

## Initialize

```
# Define circular orbit of Earth
orbit = OrbitBase.CircularOrbit(6370e+3, 3.986e+14)
```

"""
struct CircularOrbit

    _radius::Real
    _gravityconstant::Real

    # Constructor
    CircularOrbit(_radius::Real, _gravityconstant::Real) = begin
        # check if the field is appropriatelly configured
        if _radius < 0
            error("OrbitBase radius should be non-negative.")
        end
        if _gravityconstant < 0
            error("Gravity constant should be non-negative.")
        end

        new(_radius, _gravityconstant)
    end
end

"""
    function get_angular_velocity(orbit::CircularOrbit)

Calculates orbit angular velocity of a circular orbit
"""
function get_angular_velocity(orbit::CircularOrbit)::Float64

    return sqrt(orbit._gravityconstant / orbit._radius^3)

end

"""
    function get_velocity(orbit::CircularOrbit)

Calculates orbit velocity of a circular orbit. This is not angular velocity!
"""
function get_velocity(orbit::CircularOrbit)::Float64

    return sqrt(orbit._gravityconstant / orbit._radius)

end

"""
    function get_timeperiod(orbit::CircularOrbit; unit = "second")

Calculates orbit time period (second)

# Arguments

* `orbit::CircularOrbit`: struct of orbit information
* `unit`: keyword argument that defines unit of the return value
* second
* minute
"""
function get_timeperiod(orbit::CircularOrbit; unit = "second")::Float64

    if unit == "second"

        return 2 * pi / get_angular_velocity(orbit)

    elseif unit == "minute"

        return (2 * pi / get_angular_velocity(orbit))/60

    else
        error("keyword argument `unit` is set improperly.")
    end

end

"""
    setorbit(elements::OrbitalElements)::CircularOrbit

API function to define the dynamics model for the circular orbit
"""
function setorbit(elements::OrbitalElements)::CircularOrbit

    # set radius of the orbit from orbital elements
    radius = elements.semimajor_axis

    dynamicsmodel = CircularOrbit(radius, OrbitBase.EarthGravityConstant)

    return dynamicsmodel
end

end
