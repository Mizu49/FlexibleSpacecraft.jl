module OrbitBase

using ..Frames, ..DataContainers
using StaticArrays, Reexport

include("Elements.jl")
@reexport using .Elements

include("Circular.jl")
@reexport using .Circular

export OrbitInfo, OrbitData, T_RAT2LVLH, T_LVLHref2rollpitchyaw, LVLHref, setorbit

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
        orbitmodel = nothing
        planeframe = ECIframe

        return new(
            info,
            orbitmodel,
            nothing,
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
        orbitinfo = OrbitInfo(
            OrbitalElements(
                orbitparamdict["Orbital elements"]["right ascension"],
                orbitparamdict["Orbital elements"]["inclination"],
                orbitparamdict["Orbital elements"]["semimajor axis"],
                orbitparamdict["Orbital elements"]["eccentricity"],
                orbitparamdict["Orbital elements"]["argument of perigee"],
                orbitparamdict["Orbital elements"]["true anomaly at epoch"]
            ),
            ECI,
            " "
        )
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

end
