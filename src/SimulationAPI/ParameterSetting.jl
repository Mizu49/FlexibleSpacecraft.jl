module ParameterSetting

using YAML

using ..Frames
using ..Orbit
using ..RigidBody
using ..TimeLine
using ..Disturbance

export SimulationConfig, initorbitinfo, setdynamicsmodel, setsimconfig, setinitvalue, setdisturbance


"""
    struct SimulationConfig

struct that contains the information about the simulation configuration

## fields
* `simulationtime::Real`: time length of the simulation
* `samplingtime::Real`: sampling time of the simulation
"""
struct SimulationConfig
    simulationtime::Real
    samplingtime::Real
end

"""
    initorbitinfo(filepath::String, ECI::Frame)

Load the YAML file configuration and construct the appropriate model for the simulation
"""
function initorbitinfo(filepath::String, ECI::Frame)

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

"""
    setsimconfig(simulationtime::Real, samplingtime::Real)

initialize the simulation configurations

## Return value

* `simconfig::SimulationConfig`
"""
function setsimconfig(filepath::String)

    # Read configuration file
    lawread = YAML.load_file(filepath)

    if lawread["property"] != "simconfig"
        throw(AssertionError("`property` does not match with `simconfig`"))
    end

    # set values
    samplingtime = lawread["sampling time"]
    simulationtime = lawread["simulation time"]

    # struct for configurations of simulation
    simconfig = SimulationConfig(simulationtime, samplingtime)

    return simconfig
end

"""
    setinitvalue

Define the inital value for simulation
"""
function setinitvalue(filepath::String)

    lawread = YAML.load_file(filepath)

    if lawread["property"] != "initvalue"
        throw(AssertionError("`propety` does not match with `initvalue`"))
    end

    initvalue = InitData(
        lawread["quaternion"],
        lawread["angular velocity"],
        ECI_frame
    )

    return initvalue
end

"""
    setdisturbance(filepath::String)

set disturbance configuration from YAML setting file
"""
function setdisturbance(filepath::String)

    lawread = YAML.load_file(filepath)

    if lawread["property"] != "distconfig"
        throw(AssertionError("`property` deos not match with `distconfig`"))
    end

    distconfig = DisturbanceConfig(
        constanttorque = lawread["constant torque"],
        gravitygradient = lawread["gravitational torque"]
    )

    return distconfig
end

end
