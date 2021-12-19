module ParameterSetting

using YAML

import ..Frames
import ..Orbit

export initorbitinfo

function initorbitinfo(filepath::String, ECI::Frames.Frame)

    # Read YAML file
    lawdata = YAML.load_file(filepath)

    # Define Orbit.OrbitInfo
    orbitinfo = Orbit.OrbitInfo(
        Orbit.OrbitalElements(
            lawdata["OrbitalElements"]["right ascention"],
            lawdata["OrbitalElements"]["inclination"],
            lawdata["OrbitalElements"]["semimajor axis"],
            lawdata["OrbitalElements"]["eccentricity"],
            lawdata["OrbitalElements"]["argument of perigee"],
            lawdata["OrbitalElements"]["true anomaly at epoch"]
        ),
        ECI,
        lawdata["OrbitInfo"]
    )

    return orbitinfo
end


end
