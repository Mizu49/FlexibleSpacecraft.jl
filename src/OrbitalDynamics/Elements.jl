module Elements

using StaticArrays
using ..Frames, ..DataContainers, ..UtilitiesBase

export OrbitalElements, setelements

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
        if !(a >= 0) throw(DomainError(a, "Argument `a` should be positive real number.")) end
        if !(e >= 0) throw(DomainError(e, "Argument `e` should be positive real number.")) end
        if !(0 <= ω < 360) throw(DomainError(ω, "Argument `ω` exceeds appropriate domain. (0 <= ω < 360)")) end
        if !(0 <= f < 360) throw(DomainError(f, "Argument `f` exceeds appropriate domain. (0 <= ω < 360)")) end

        new(Ω, i, a, e, ω, f)
    end
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
