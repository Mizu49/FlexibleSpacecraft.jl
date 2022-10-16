module OrbitBase

using ..Frames, ..DataContainers, ..UtilitiesBase
using StaticArrays, Reexport, LinearAlgebra

const GravityConstant = 6.673e-11
const EarthMass = 5.974e24
const EarthGravityConstant = GravityConstant * EarthMass

include("Elements.jl")
@reexport using .Elements

include("NoOrbit.jl")
@reexport using .NoOrbit

include("Circular.jl")
@reexport using .Circular

export OrbitInfo, OrbitData, initorbitdata, T_UnitFrame2LVLHFrame, LVLHUnitFrame, T_RAT2LVLH, T_LVLH2RPY, setorbit, get_angular_velocity

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

    orbitalmodel = orbitparamdict["Dynamics model"]

    if orbitalmodel == "none"

        elements = OrbitalElements(0, 0, 0, 0, 0, 0)
        orbitinfo = OrbitInfo(NoOrbitModel(), elements, ECI, info = "no orbit simulation");

    elseif orbitalmodel == "Circular"
        # set the orbital parameter for the circular orbit

        elements = setelements(orbitparamdict["Orbital elements"])
        orbitmodel = Circular.setorbit(elements)
        orbitalplaneframe = calc_orbitalframe(elements, ECI)

        orbitinfo = OrbitInfo(orbitmodel, elements, orbitalplaneframe)
    else
        error("orbital model \"$orbitalmodel\" not found")
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
    transformation matrix from unit frame to LVLH referential frame
"""
const T_UnitFrame2LVLHFrame = diagm([1, -1, -1])

"""
    LVLH referential frame
"""
const LVLHUnitFrame = Frame([1, 0, 0], [0, -1, 0], [0, 0, -1])

"""
    rotational matrix that transfers from radial-along-track (RAT) to local-vertical local-horizontal (LVLH) attitude representation
"""
T_RAT2LVLH = C1(-pi/2) * C3(pi/2)

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
    NoOrbit.get_timeperiod(orbitmodel, unit = unit)
end

end
