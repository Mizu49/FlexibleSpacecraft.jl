module Elements

using StaticArrays
using ..Frames, ..DataContainers

export OrbitalElements, ECI2OrbitalPlaneFrame, OrbitalPlaneFrame2RadialAlongTrack, OrbitalPlaneFrame2LVLH, calc_orbitalframe, update_radial_along_track, setelements

"""
    struct OrbitalElements

Struct of orbital elements (keplerian elements)

# Constructor

`OrbitalElements(Ω, i, a, e, ω, f)`

* `Ω`: right ascention of ascending node (degree)
* `i`: inclination (degree)
* `a`: semimajor axis (m)
* `e`: eccentricity
* `ω`: argument of perigee (degree)
* `f`: true anomaly at epoch(degree)

"""
struct OrbitalElements

    # Orientation of the orbital plane
    ascention
    inclination

    # Shape of the ellipse
    semimajor_axis
    eccentricity

    # Defines the orientation of the ellipse in the orbital plane
    arg_perigee

    # Reference point on orbit
    true_anomaly

    # Constructor
    OrbitalElements(Ω, i, a, e, ω, f) = begin

        # Check if the arguments are appropriate
        if !(0 <= Ω < 360) throw(DomainError(Ω, "Argument `Ω` exceeds appropriate domain. (0 <= Ω < 360)")) end
        if !(0 <= i < 360) throw(DomainError(i, "Argument `i` exceeds appropriate domain. (0 <= i < 360)")) end
        if !(a > 0) throw(DomainError(a, "Argument `a` should be positive real number.")) end
        if !(e >= 0) throw(DomainError(e, "Argument `e` should be positive real number.")) end
        if !(0 <= ω < 360) throw(DomainError(ω, "Argument `ω` exceeds appropriate domain. (0 <= ω < 360)")) end
        if !(0 <= f < 360) throw(DomainError(f, "Argument `f` exceeds appropriate domain. (0 <= ω < 360)")) end

        new(Ω, i, a, e, ω, f)
    end
end

"""
    function ECI2OrbitalPlaneFrame(elements::OrbitalElements)
"""
function ECI2OrbitalPlaneFrame(elements::OrbitalElements)
    return C1(elements.inclination) * C3(elements.ascention)
end

function ECI2OrbitalPlaneFrame(elements::Nothing)
    return C1(0) * C3(0)
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

"""
    C1(theta::Real)::SMatrix

Rotational matrix for 1-axis
"""
function C1(theta::Real)::SMatrix
    return SMatrix{3, 3, <:Real}([
        1 0 0
        0 cos(theta) sin(theta)
        0 -sin(theta) cos(theta)
    ])
end

"""
    C2(theta::Real)::SMatrix

Rotational matrix for 2-axis
"""
function C2(theta::Real)::SMatrix
    return SMatrix{3, 3, <:Real}([
        cos(theta) 0 -sin(theta)
        0 1 0
        sin(theta) 0 cos(theta)
    ])
end

"""
    C3(theta::Real)::SMatrix

Rotational matrix for 3-axis
"""
function C3(theta::Real)::SMatrix
    return SMatrix{3, 3, <:Real}([
        cos(theta) sin(theta) 0
        -sin(theta) cos(theta) 0
        0 0 1
    ])
end

function setelements(paramdict::AbstractDict)::OrbitalElements

    return OrbitalElements(
        paramdict["right ascension"],
        paramdict["inclination"],
        paramdict["semimajor axis"],
        paramdict["eccentricity"],
        paramdict["argument of perigee"],
        paramdict["true anomaly at epoch"]
    )
end

end
