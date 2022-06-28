module OrbitBase

using ..Frames, ..DataContainers
using StaticArrays, Reexport

include("Elements.jl")
@reexport using .Elements

include("NoOrbit.jl")
@reexport using .NoOrbit

include("Circular.jl")
@reexport using .Circular

export OrbitInfo, OrbitData, T_RAT2LVLH, T_LVLHref2rollpitchyaw, LVLHref, setorbit, get_angular_velocity

"""
    OrbitInfo

struct that contains the information about the orbital dynamics of the spacecraft
"""
struct OrbitInfo
    info
    orbitmodel
    orbitalelement
    planeframe

    OrbitInfo(orbitalelement::OrbitalElements, ECIframe::Frame, info::String) = begin

        orbitmodel = CircularOrbit(6370e+3 + 400e3, 3.986e+14)

        planeframe = calc_orbitalframe(orbitalelement, ECIframe)

        return new(
            info,
            orbitmodel,
            orbitalelement,
            planeframe
        )
    end

    OrbitInfo(orbitalelement::Nothing, ECIframe, info::String) = begin
        orbitmodel = NoOrbitModel()
        planeframe = ECIframe

        return new(
            info,
            orbitmodel,
            orbitalelement,
            planeframe
        )
    end
end

"""
    setorbit

Load the configuration from YAML file and construct the appropriate model for the simulation. Works with the `ParameterSettingBase.jl`.
"""
function setorbit(orbitparamdict::AbstractDict, ECI::Frame)::OrbitInfo

    if orbitparamdict["Dynamics model"] == "none"

        orbitinfo = OrbitInfo(
            nothing,
            ECI,
            "no orbit simulation"
        );

    elseif orbitparamdict["Dynamics model"] == "Circular"
        # set the orbital parameter for the circular orbit

        elements = setelements(orbitparamdict["Orbital elements"])
        orbitinfo = OrbitInfo(elements, ECI, " ")
    end

    return orbitinfo
end

"""
    OrbitData

struct of the data containers for the orbital motion

# Fields

* `angularposition::Vector{<:Real}`: angular position of the orbital motion of the spacecraft
* `angularvelocity::Vector{<:Real}`: angular velocity of the orbital motion of the spacecraft
* `LVLH::StructArray`: data container for the time history of the LVLH frame

"""
struct OrbitData
    angularposition::Vector{<:Real}
    angularvelocity::Vector{<:Real}
    LVLH::Vector{<:Frame}
end

function initorbitdata(datanum::Integer, orbitalframe::Frame)

    return OrbitData(
        zeros(datanum),
        zeros(datanum),
        initframes(datanum, orbitalframe)
    )
end

"""
    T_RAT2LVLH

Transformation matrix from radial along track frame to LVLH frame. LVLH frame is defined with the replacement of the coordinate of the radial-along-track frame.
"""
const T_RAT2LVLH = SMatrix{3, 3}([0 1 0; 0 0 -1; 1 0 0])

"""
    T_LVLHref2rollpitchyaw

Transformation matrix from LVLH reference frame to roll-pitch-yaw frame. This transformation matrix converts the reference frame from `UnitFrame` to `LVLHref`.
"""
const T_LVLHref2rollpitchyaw = SMatrix{3, 3}([0 1 0; 0 0 1; -1 0 0])

"""
    LVLHref

    Reference unit frame for the LVLH frame
"""
const LVLHref = Frame([1, 0, 0], [0, -1, 0], [0, 0, -1])

const AbstractOrbitModel = Union{NoOrbitModel, CircularOrbit}

function get_angular_velocity(orbitmodel::CircularOrbit)
    Circular.get_angular_velocity(orbitmodel)
end

function get_angular_velocity(orbitmodel::NoOrbitModel)
    NoOrbit.get_angular_velocity(orbitmodel)
end

function get_velocity(orbitmodel::CircularOrbit)
    CircularOrbit.get_velocity(orbitmodel)
end

function get_velocity(orbitmodel::NoOrbitModel)
    NoOrbit.get_velocity(orbitmodel)
end

function get_timeperiod(orbitmodel::CircularOrbit; unit = "second")
    CircularOrbit.get_timeperiod(orbitmodel, unit)
end

function get_timeperiod(orbitmodel::NoOrbitModel; unit = "second")
    NoOrbit.get_timeperiod(orbitmodel, unit)
end

end
