module OrbitalFrames

using StaticArrays, LinearAlgebra
using ..OrbitBase, ..Elements
using ..Frames, ..DataContainers, ..UtilitiesBase

export ECI2OrbitalPlane, ECI2ORF, OrbitalPlane2RadialAlongTrack, calc_orbitalframe, calc_inital_quaternion

"""
    function ECI2OrbitalPlane(elements::OrbitalElements)
"""
function ECI2OrbitalPlane(elements::OrbitalElements)
    return C1(deg2rad(elements.inclination)) * C3(deg2rad(elements.ascention))
end

function ECI2OrbitalPlane(elements::Nothing)
    return C1(0) * C3(0)
end

function ECI2ORF(elements::OrbitalElements, orbitposition::Real)

    C_ECI2OrbitPlane = ECI2OrbitalPlane(elements)
    C_OrbitalPlane2RAT = OrbitalPlane2RadialAlongTrack(elements, orbitposition)
    C_ECI2ORF = C_OrbitalPlane2RAT * C_ECI2OrbitPlane

    return C_ECI2ORF
end

"""
    OrbitalPlane2RadialAlongTrack(elements::OrbitalElements, orbitalposition::Real)

Calculates transformation matrix from OrbitalPlaneFrame to Radial Along Track frame
"""
function OrbitalPlane2RadialAlongTrack(elements::OrbitalElements, orbitalposition::Real)

    # current angle of spacecraft relative to ascending axis of orbital plane frame
    current_position = deg2rad(elements.true_anomaly) + orbitalposition

    # calculate the radial along track
    return C2(-current_position)
end

function OrbitalPlane2RadialAlongTrack(elements::Nothing, orbitalposition::Real)
    return C2(0)
end

function calc_orbitalframe(elem::OrbitalElements, ECI_frame::Frame)::Frame

    C = ECI2OrbitalPlane(elem)

    return C * ECI_frame
end

function calc_inital_quaternion(elements::OrbitalElements, initRPY::AbstractVector{<:Real})::SVector{4, <:Real}

    C_ECI2OrbitPlane = ECI2OrbitalPlane(elements)
    C_OrbitalPlane2RAT = OrbitalPlane2RadialAlongTrack(elements, 0)
    C_LVLH2BRF = euler2dcm(initRPY)

    initquaternion = dcm2quaternion(C_LVLH2BRF * C_OrbitalPlane2RAT * C_ECI2OrbitPlane)

    return initquaternion
end

end
