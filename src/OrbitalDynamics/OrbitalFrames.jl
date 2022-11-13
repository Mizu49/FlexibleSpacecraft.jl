module OrbitalFrames

using StaticArrays
using ..OrbitBase, ..Elements
using ..Frames, ..DataContainers, ..UtilitiesBase

export ECI2OrbitalPlane, ECI2LVLH, OrbitalPlane2RadialAlongTrack, calc_orbitalframe

"""
    function ECI2OrbitalPlane(elements::OrbitalElements)
"""
function ECI2OrbitalPlane(elements::OrbitalElements)
    return C1(elements.inclination) * C3(elements.ascention)
end

function ECI2OrbitalPlane(elements::Nothing)
    return C1(0) * C3(0)
end

function ECI2LVLH(elements::OrbitalElements, orbitposition::Real)

    C_ECI2OrbitPlane = ECI2OrbitalPlane(elements)

    M = SMatrix{3, 3}([
        0  1  0
        0  0 -1
        -1 0  0
    ])

    C_ECI2ORF = M * C3(orbitposition) * C_ECI2OrbitPlane

    return SMatrix{3, 3}(C_ECI2ORF)
end

"""
    OrbitalPlane2RadialAlongTrack(elements::OrbitalElements, orbitalposition::Real)

Calculates transformation matrix from OrbitalPlaneFrame to Radial Along Track frame
"""
function OrbitalPlane2RadialAlongTrack(elements::OrbitalElements, orbitalposition::Real)

    # current angle of spacecraft relative to ascending axis of orbital plane frame
    current_position = deg2rad(elements.true_anomaly) + orbitalposition

    return C3(current_position)
end

function OrbitalPlane2RadialAlongTrack(elements::Nothing, orbitalposition::Real)
    return C3(0)
end

function calc_orbitalframe(elem::OrbitalElements, ECI_frame::Frame)::Frame

    C = ECI2OrbitalPlane(elem)

    return C * ECI_frame
end

end
