module OrbitBase

using ..Frames, ..DataContainers
using StaticArrays, Reexport

const GravityConstant = 6.673e-11
const EarthMass = 5.974e24
const EarthGravityConstant = GravityConstant * EarthMass

include("Elements.jl")
@reexport using .Elements

include("NoOrbit.jl")
@reexport using .NoOrbit

include("Circular.jl")
@reexport using .Circular

export OrbitInfo, OrbitData, T_RAT2LVLH, T_LVLHref2rollpitchyaw, LVLHref, setorbit, get_angular_velocity

const AbstractOrbitModel = Union{NoOrbitModel, CircularOrbit}

"""
    OrbitInfo

struct that contains the information about the orbital dynamics of the spacecraft
"""
struct OrbitInfo
    orbitmodel::AbstractOrbitModel
    orbitalelement::OrbitalElements
    planeframe::Frame
    info::String

    OrbitInfo(orbitmodel::AbstractOrbitModel, orbitalelement::OrbitalElements, planeframe::Frame; info::String = "") = begin
        new(orbitmodel, orbitalelement, planeframe, info)
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
        orbitmodel = Circular.setorbit(elements)
        orbitalplaneframe = calc_orbitalframe(elements, ECI)

        orbitinfo = OrbitInfo(orbitmodel, elements, orbitalplaneframe)
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
