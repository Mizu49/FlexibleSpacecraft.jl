module OrbitalFrames

using StaticArrays
using ..OrbitBase, ..Elements
using ..Frames, ..DataContainers, ..UtilitiesBase

export ECI2OrbitalPlaneFrame, ECI2LVLH, OrbitalPlaneFrame2RadialAlongTrack, OrbitalPlaneFrame2LVLH, calc_orbitalframe, update_radial_along_track

"""
    function ECI2OrbitalPlaneFrame(elements::OrbitalElements)
"""
function ECI2OrbitalPlaneFrame(elements::OrbitalElements)
    return C1(elements.inclination) * C3(elements.ascention)
end

function ECI2OrbitalPlaneFrame(elements::Nothing)
    return C1(0) * C3(0)
end

function ECI2LVLH(elements::OrbitalElements, orbitposition::Real)

    C_ECI2OrbitPlane = ECI2OrbitalPlaneFrame(elements)

    M = SMatrix{3, 3}([
        0  1  0
        0  0 -1
        -1 0  0
    ])

    C_ECI2ORF = M * C3(orbitposition) * C_ECI2OrbitPlane

    return SMatrix{3, 3}(C_ECI2ORF)
end

"""
    function OrbitalPlaneFrame2RadialAlongTrack(elements::OrbitalElements, angular_velocity, time)

Calculates transformation matrix from OrbitalPlaneFrame to Radial Along Track frame
"""
function OrbitalPlaneFrame2RadialAlongTrack(elements::OrbitalElements, angular_velocity, time)

    # current angle of spacecraft relative to ascending axis of orbital plane frame
    current_position = deg2rad(elements.true_anomaly) + angular_velocity * time

    return C3(current_position)
end

function OrbitalPlaneFrame2RadialAlongTrack(elements::Nothing, angular_velocity::Real, time::Real)

    # current angle of spacecraft relative to ascending axis of orbital plane frame
    current_position = 0

    return C3(current_position)
end


"""
    function OrbitalPlaneFrame2LVLH(OrbitalPlaneFrame2RadialAlongTrack)

Calculates transformation matrix from OrbitalPlaneFrame frame to LVLH
"""
function OrbitalPlaneFrame2LVLH(C_OrbitalPlaneFrame2RadialAlongTrack)

    C = [  C_OrbitalPlaneFrame2RadialAlongTrack[2,:]';
          -C_OrbitalPlaneFrame2RadialAlongTrack[3,:]';
          -C_OrbitalPlaneFrame2RadialAlongTrack[1,:]']

    return C

end

function calc_orbitalframe(elem::OrbitalElements, ECI_frame::Frame)::Frame

    C = ECI2OrbitalPlaneFrame(elem)

    return C * ECI_frame
end

function update_radial_along_track(orbitframe::Frame, elem::OrbitalElements, time::Real, angularvelocity::Real)::Frame

    C_RAT = OrbitalPlaneFrame2RadialAlongTrack(elem, angularvelocity, time)

    return C_RAT * orbitframe
end

end
