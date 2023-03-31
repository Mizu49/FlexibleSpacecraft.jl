module OrbitBase

using ..Frames, ..DataContainers, ..UtilitiesBase
using StaticArrays, Reexport, LinearAlgebra

const GravityConstant = 6.673e-11
const EarthMass = 5.974e24
const EarthGravityConstant = GravityConstant * EarthMass

abstract type AbstractOrbitalDynamics end

export AbstractOrbitalDynamics, OrbitInfo, OrbitData, OrbitInternals, initorbitdata, T_UnitFrame2LVLHFrame, LVLHUnitFrame, T_RAT2LVLH, T_LVLH2RPY, setorbit, update_orbitstate!

include("Elements.jl")
using .Elements

include("Circular.jl")
using .Circular

include("OrbitalFrames.jl")
@reexport using .OrbitalFrames

"""
    OrbitInternals

struct that contains the information about the current state of the orbital dynamics
"""
mutable struct OrbitInternals
    angularposition::Float64
    angularvelocity::Float64
end

"""
    OrbitInfo

struct that contains the information about the orbital dynamics of the spacecraft
"""
struct OrbitInfo
    dynamicsmodel::AbstractOrbitalDynamics
    orbitalelement::OrbitalElements
    internals::OrbitInternals
    planeframe::Frame
    info::String

    # Constructor
    OrbitInfo(
        dynamicsmodel::AbstractOrbitalDynamics,
        orbitalelement::OrbitalElements,
        internals::OrbitInternals,
        planeframe::Frame;
        info::String = ""
        )= begin
            new(dynamicsmodel, orbitalelement, internals, planeframe, info)
    end
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

"""
    initorbitdata

initialize data container for orbital dynamics
"""
function initorbitdata(datanum::Integer, orbitinfo::OrbitInfo)::OrbitData

    return OrbitData(
        zeros(datanum),
        zeros(datanum),
        initframes(datanum, orbitinfo.planeframe)
    )
end

"""
    setorbit

Load the configuration from YAML file and construct the appropriate model for the simulation. Works with the `ParameterSettingBase.jl`.
"""
function setorbit(orbitparamdict::AbstractDict, ECI::Frame)::Union{OrbitInfo, Nothing}

    orbitalmodel = orbitparamdict["Dynamics model"]

    if orbitalmodel == "none"

        return nothing

    elseif orbitalmodel == "Circular"
        # set the orbital parameter for the circular orbit
        elements = Elements.setelements(orbitparamdict["Orbital elements"])
        dynamicsmodel = Circular.setorbit(elements)
        orbitalplaneframe = OrbitalFrames.calc_orbitalframe(elements, ECI)
        orbitinternals = OrbitInternals(0, 0)

        orbitinfo = OrbitInfo(dynamicsmodel, elements, orbitinternals, orbitalplaneframe)

        return orbitinfo
    else
        error("orbital model \"$orbitalmodel\" not found")
    end
end

function _update_orbitinternals!(orbitinternals::OrbitInternals, angularvelocity::Real, angularposition::Real)::Nothing

    orbitinternals.angularvelocity = angularvelocity
    orbitinternals.angularposition = angularposition

    return
end

function update_orbitstate!(orbitinfo::OrbitInfo, currenttime::Real)

    angularvelocity = _get_angularvelocity(orbitinfo.dynamicsmodel)
    angularposition = angularvelocity * currenttime

    _update_orbitinternals!(orbitinfo.internals, angularvelocity, angularposition)

    C_ECI2LVLH = ECI2ORF(orbitinfo.orbitalelement, angularposition)

    return C_ECI2LVLH
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
