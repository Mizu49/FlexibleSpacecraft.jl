module ParameterSetting

using YAML

import ..Frames
import ..Orbit
import ..RigidBody

export initorbitinfo, setdynamicsmodel

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

"""
    setdynamicsmodel(filepath::String)

Load the YAML file configuration and construct the appropriate model for the simulation
"""
function setdynamicsmodel(filepath::String)

    lawdata = YAML.load_file(filepath)

    if lawdata["dynamicsmodel"] == "Rigid body"
        inertia = reshape(lawdata["platform"]["inertia"], (3,3))

        model = RigidBody.RigidBodyModel(inertia)
    else
        error("configuration for dynamics model in YAML file is set improperly")
    end

    return model
end

end
