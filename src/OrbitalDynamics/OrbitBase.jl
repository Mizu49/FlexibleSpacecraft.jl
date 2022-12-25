module OrbitBase

using ..Frames, ..DataContainers, ..UtilitiesBase
using StaticArrays, Reexport, LinearAlgebra

const GravityConstant = 6.673e-11
const EarthMass = 5.974e24
const EarthGravityConstant = GravityConstant * EarthMass

abstract type AbstractOrbitalDynamics end

export AbstractOrbitalDynamics, OrbitInfo, OrbitData, OrbitInternals, initorbitdata, T_UnitFrame2LVLHFrame, LVLHUnitFrame, T_RAT2LVLH, T_LVLH2RPY, setorbit, update_orbitstate!

include("Elements.jl")
@reexport using .Elements

include("OrbitalFrames.jl")
@reexport using .OrbitalFrames

include("Circular.jl")
@reexport using .Circular

"""
    OrbitInfo

struct that contains the information about the orbital dynamics of the spacecraft
"""
struct OrbitInfo
    dynamicsmodel::AbstractOrbitalDynamics
    orbitalelement::OrbitalElements
    planeframe::Frame
    info::String

    OrbitInfo(dynamicsmodel::AbstractOrbitalDynamics, orbitalelement::OrbitalElements, planeframe::Frame; info::String = "") = begin
        new(dynamicsmodel, orbitalelement, planeframe, info)
    end
end

"""
    OrbitInternals

struct that contains the information about the current state of the orbital dynamics
"""
mutable struct OrbitInternals
    angularposition::Float64
    angularvelocity::Float64
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

function initorbitdata(datanum::Integer, orbitalframe::Frame)::OrbitData

    return OrbitData(
        zeros(datanum),
        zeros(datanum),
        initframes(datanum, orbitalframe)
    )
end

"""
    setorbit

Load the configuration from YAML file and construct the appropriate model for the simulation. Works with the `ParameterSettingBase.jl`.
"""
function setorbit(orbitparamdict::AbstractDict, ECI::Frame)

    orbitalmodel = orbitparamdict["Dynamics model"]

    if orbitalmodel == "none"

        return nothing

    elseif orbitalmodel == "Circular"
        # set the orbital parameter for the circular orbit

        elements = setelements(orbitparamdict["Orbital elements"])
        dynamicsmodel = Circular.setorbit(elements)
        orbitalplaneframe = calc_orbitalframe(elements, ECI)

        orbitinternals = OrbitInternals(0, 0)

        orbitinfo = OrbitInfo(dynamicsmodel, elements, orbitalplaneframe)

        return (orbitinfo, orbitinternals)
    else
        error("orbital model \"$orbitalmodel\" not found")
        return
    end
end

function _update_orbitinternals!(orbitinternals::OrbitInternals, angularvelocity::Real, angularposition::Real)::Nothing

    orbitinternals.angularvelocity = angularvelocity
    orbitinternals.angularposition = angularposition

    return
end

function update_orbitstate!(orbitinfo::OrbitInfo, orbitinternals::OrbitInternals, currenttime::Real)::Tuple

    angularvelocity = _get_angularvelocity(orbitinfo.dynamicsmodel)
    angularposition = angularvelocity * currenttime

    _update_orbitinternals!(orbitinternals, angularvelocity, angularposition)

    return (angularvelocity, angularposition)
end

"""
    transformation matrix from unit frame to LVLH referential frame
"""
const T_UnitFrame2LVLHFrame = diagm([1.0, -1.0, -1.0])

"""
    LVLH referential frame
"""
const LVLHUnitFrame = Frame([1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0])

"""
    rotational matrix that transfers from radial-along-track (RAT) to local-vertical local-horizontal (LVLH) attitude representation
"""
T_RAT2LVLH = C1(-pi/2) * C3(pi/2)

function _get_angularvelocity(dynamicsmodel::CircularOrbit)::Float64
    Circular.get_angular_velocity(dynamicsmodel)
end

function _get_angularvelocity(dynamicsmodel::Nothing)::Float64
    return 0.0
end

function get_velocity(dynamicsmodel::CircularOrbit)::Float64
    CircularOrbit.get_velocity(dynamicsmodel)
end

function get_velocity(dynamicsmodel::Nothing)::Float64
    return 0.0
end

function get_timeperiod(dynamicsmodel::CircularOrbit; unit = "second")::Float64
    timeperiod = CircularOrbit.get_timeperiod(dynamicsmodel, unit)
    return timeperiod
end

function get_timeperiod(dynamicsmodel::Nothing; unit = "second")::Float64
    return 0.0
end

end
